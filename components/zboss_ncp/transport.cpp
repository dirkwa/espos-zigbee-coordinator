#include "transport.h"
#include "app.h"
#include "protocol.h"
#include "utils.h"
#include "zb_ncp.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <cerrno>
#include <cstring>
#include <initializer_list>

static const char* TAG = "TRNPT";

// How long one frame may take to leave. The radio is shared with the
// always-listening coordinator, so Wi-Fi can be starved for seconds, and the
// app task that sends also serves every Zigbee indication. Each ::send blocks
// at most SEND_SLICE_MS (SO_SNDTIMEO), so the loop below can stop at this
// bound rather than a whole socket timeout past it.
static constexpr uint32_t TCP_FRAME_DEADLINE_MS = 8000;
static constexpr uint32_t SEND_SLICE_MS = 500;

// A second host must send a ZBOSS frame within this long to take over.
static constexpr int64_t QUALIFY_TIMEOUT_US = 10LL * 1000 * 1000;

// The framing-level ACK herdsman's open-port handshake waits for
// (andryblack/esp-coordinator#11). A host connects whenever it likes and may
// reconnect, so every host gets it, not only the first.
static const uint8_t BOOT_ACK[] = {0xDE, 0xAD, 0x05, 0x00, 0x06, 0x01, 0x8F};

transport::transport() {}

transport& transport::instance() {
	static transport s_transport;
	return s_transport;
}

esp_err_t transport::write_int(const void *buffer, size_t size)
{
    utils::sem_lock l(m_client_sem);
    const int fd = m_client.load(std::memory_order_relaxed);
    if (fd < 0) {
        // No host: the frame has nobody to go to, which is not an error the
        // app task should log once per frame.
        return ESP_OK;
    }
    const auto* p = static_cast<const uint8_t*>(buffer);
    size_t off = 0;
    const int64_t deadline = esp_timer_get_time() + int64_t(TCP_FRAME_DEADLINE_MS) * 1000;
    while (off < size && esp_timer_get_time() < deadline) {
        int n = ::send(fd, p + off, size - off, 0);
        if (n > 0) {
            off += n;
        } else if (n == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) {
            break;
        }
    }
    if (off == size) {
        return ESP_OK;
    }
    // A frame cut short leaves the host's parser mid-frame, so the link
    // cannot carry on: end it, and the host reconnects to fresh boot frames.
    ESP_LOGE(TAG, "send stalled at %u/%u bytes (errno %d), closing the host link",
             unsigned(off), unsigned(size), errno);
    ::shutdown(fd, SHUT_RDWR);
    return ESP_FAIL;
}

void transport::send_direct(int fd, const void* data, size_t size) {
    // Boot frames on a socket that is not routed yet. Small and the first
    // bytes on a fresh connection, so one bounded send takes them.
    if (::send(fd, data, size, 0) != int(size)) {
        ESP_LOGW(TAG, "boot frame not sent (errno %d)", errno);
    }
}

void transport::rx_pump(const uint8_t* data, int readed) {
    app::ctx_t ncp_event = {
        .event = app::EVENT_OUTPUT,
        .size = 0
    };

    // TRNPT-2: xStreamBufferSend can accept FEWER than `readed` bytes when
    // m_output_buf is near-full. Posting EVENT_OUTPUT with the full read
    // size then over-claims and output_receive desyncs the byte stream.
    // Post only what was actually committed, and push the remainder under a
    // bounded wait so transient backpressure doesn't silently drop host
    // bytes. Splitting one read into several EVENT_OUTPUT posts is safe —
    // the protocol layer reassembles frames across on_rx calls.
    utils::sem_lock l(m_output_sem);
    int off = 0;
    while (off < readed) {
        size_t sent = xStreamBufferSend(m_output_buf, data + off,
                                        readed - off,
                                        pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS));
        if (sent == 0) {
            ESP_LOGE(TAG, "output_buf full, dropping %d bytes", readed - off);
            break;
        }
        ncp_event.size = sent;
        // Bounded retry on a momentarily-full event queue: the bytes are
        // already committed to m_output_buf, so a dropped event would orphan
        // them and shift the byte/event pairing. Retrying HERE is safe —
        // rx_pump only ever runs on the server task, never on the ZBOSS task
        // (app::send_event itself must stay non-blocking for that caller —
        // see the critical-section panic note there).
        int tries = RINGBUF_TIMEOUT_MS / 10;
        while (app::send_event(ncp_event) != ESP_OK && tries-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        off += sent;
    }
}

void transport::close_client_int() {
    // No semaphore: a stalled send can hold it for seconds, and this runs on
    // the restart path, sometimes from the esp_timer task. shutdown is safe
    // beside a send on the same socket; only the server task closes it.
    const int fd = m_client.load(std::memory_order_relaxed);
    if (fd >= 0) {
        ::shutdown(fd, SHUT_RDWR);
    }
}

void transport::drop_fd(int fd) {
    utils::sem_lock l(m_client_sem);
    if (m_client.load(std::memory_order_relaxed) == fd) {
        m_client.store(-1, std::memory_order_relaxed);
    }
    if (m_switching == fd) {
        m_switching = -1;
    }
    ::close(fd);
}

void transport::promote(int fd) {
    {
        utils::sem_lock l(m_client_sem);
        m_switching = fd;
    }
    // The fd rides in the event so that a host replaced or gone before the
    // app task gets to it is not switched in: on_link_reset checks it is
    // still the one switching.
    app::ctx_t ev = {.event = app::EVENT_LINK_RESET, .size = uint16_t(fd)};
    while (app::send_event(ev) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "host connected");
}

void transport::on_link_reset_int(int fd) {
    {
        utils::sem_lock l(m_client_sem);
        if (m_switching != fd) {
            return;
        }
        // Frames queued from here on were stamped for the previous host.
        m_client.store(-1, std::memory_order_relaxed);
    }
    // The new host's sequence numbers start at 0, so ours restart as after a
    // boot.
    protocol::reset_link_state();
    {
        utils::sem_lock l(m_client_sem);
        if (m_switching != fd) {
            return;
        }
        send_direct(fd, BOOT_ACK, sizeof(BOOT_ACK));
        // Before the stack has loaded its NVRAM the boot-ready frame would let
        // the host read default PAN values and form a new network over the
        // stored one (andryblack/esp-coordinator#5); on_boot_ready sends it
        // once the dataset is loaded.
        if (m_boot_ready) {
            size_t n = 0;
            const uint8_t* frame = zb_ncp::boot_ready_frame(&n);
            send_direct(fd, frame, n);
        }
    }
    // Behind everything already queued, so all of that is dropped, and ahead
    // of anything the new host's first request produces.
    app::ctx_t ev = {.event = app::EVENT_LINK_ACTIVE, .size = uint16_t(fd)};
    while (app::send_event(ev) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void transport::on_link_active_int(int fd) {
    utils::sem_lock l(m_client_sem);
    if (m_switching == fd) {
        m_client.store(fd, std::memory_order_relaxed);
        m_switching = -1;
    }
}

void transport::on_boot_ready_int() {
    m_boot_ready = true;
    utils::sem_lock l(m_client_sem);
    size_t n = 0;
    const uint8_t* frame = zb_ncp::boot_ready_frame(&n);
    const int fd = m_client.load(std::memory_order_relaxed);
    if (fd >= 0) {
        send_direct(fd, frame, n);
    }
    // A host still between reset and active has had its boot frames already,
    // without this one: it goes to that socket now.
    if (m_switching >= 0) {
        send_direct(m_switching, frame, n);
    }
}

void transport::restart() {
    close_client();
    // Long enough for the FIN to leave while the radio is still up.
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

static void set_client_options(int fd) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    // lwIP's default keepalive idle is two hours, so a host that vanished
    // without a FIN (a crash, a pulled cable) would hold the only host slot
    // that long. 15 s matches herdsman's own setKeepAlive; three missed probes
    // 3 s apart then end the connection.
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    int idle = 15, intvl = 3, cnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
    struct timeval to = {};
    to.tv_usec = SEND_SLICE_MS * 1000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
}

// Index of the first byte of a DE AD signature in buf, -1 when it began with
// the 0xDE that ended the previous chunk, -2 when there is none yet.
// `pending_de` carries that last-byte state across chunks.
static int find_signature(const uint8_t* buf, int len, bool& pending_de) {
    if (pending_de && len > 0 && buf[0] == 0xAD) {
        pending_de = false;
        return -1;
    }
    for (int i = 0; i + 1 < len; ++i) {
        if (buf[i] == 0xDE && buf[i + 1] == 0xAD) {
            pending_de = false;
            return i;
        }
    }
    pending_de = (len > 0 && buf[len - 1] == 0xDE);
    return -2;
}

void transport::task_int() {
    m_listen = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_listen < 0) {
        ESP_LOGE(TAG, "socket() failed (errno %d)", errno);
        vTaskDelete(NULL);
        return;
    }
    int one = 1;
    setsockopt(m_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(CONFIG_ZBOSS_NCP_TCP_PORT);
    if (::bind(m_listen, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) != 0 ||
        ::listen(m_listen, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen :%d failed (errno %d)", CONFIG_ZBOSS_NCP_TCP_PORT, errno);
        ::close(m_listen);
        m_listen = -1;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "NCP listening on tcp :%d", CONFIG_ZBOSS_NCP_TCP_PORT);

    int host = -1;           // the host, promoted
    int cand = -1;           // a second connection that has not sent DE AD yet
    bool cand_de = false;
    int64_t cand_since = 0;
    while (true) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_listen, &rfds);
        int maxfd = m_listen;
        for (int fd : {host, cand}) {
            if (fd >= 0) {
                FD_SET(fd, &rfds);
                if (fd > maxfd) maxfd = fd;
            }
        }
        struct timeval tv = {};
        tv.tv_sec = 1;
        int ready = ::select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno != EINTR) vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (cand >= 0 && esp_timer_get_time() - cand_since > QUALIFY_TIMEOUT_US) {
            ESP_LOGW(TAG, "second connection sent no ZBOSS frame, dropped");
            ::close(cand);
            cand = -1;
        }

        if (FD_ISSET(m_listen, &rfds)) {
            int fd = ::accept(m_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_client_options(fd);
                if (host < 0) {
                    // Nobody to protect: a host that connects to an idle
                    // coordinator is the host.
                    host = fd;
                    promote(host);
                } else {
                    if (cand >= 0) {
                        ::close(cand);
                    }
                    cand = fd;
                    cand_de = false;
                    cand_since = esp_timer_get_time();
                }
            }
        }

        if (cand >= 0 && FD_ISSET(cand, &rfds)) {
            int n = ::recv(cand, m_temp_buf, BUF_SIZE, 0);
            if (n <= 0 && !(n < 0 && (errno == EINTR || errno == EAGAIN))) {
                ::close(cand);
                cand = -1;
            } else if (n > 0) {
                int sig = find_signature(m_temp_buf, n, cand_de);
                if (sig != -2) {
                    // A real host: a ZBOSS host reconnecting from a session
                    // this end still holds, half open, takes over now instead
                    // of after the keepalive gives up on the old one.
                    ESP_LOGW(TAG, "new host replaces the connected one");
                    drop_fd(host);
                    host = cand;
                    cand = -1;
                    promote(host);
                    if (sig == -1) {
                        const uint8_t de = 0xDE;
                        rx_pump(&de, 1);
                        rx_pump(m_temp_buf, n);
                    } else {
                        rx_pump(m_temp_buf + sig, n - sig);
                    }
                }
            }
        }

        if (host >= 0 && FD_ISSET(host, &rfds)) {
            int n = ::recv(host, m_temp_buf, BUF_SIZE, 0);
            if (n > 0) {
                rx_pump(m_temp_buf, n);
            } else if (!(n < 0 && (errno == EINTR || errno == EAGAIN))) {
                ESP_LOGW(TAG, "host disconnected");
                drop_fd(host);
                host = -1;
                // A connection that waited for the slot has it now.
                if (cand >= 0) {
                    host = cand;
                    cand = -1;
                    promote(host);
                }
            }
        }
    }
}

size_t transport::output_receive(void* buffer, size_t size) {
	return xStreamBufferReceive(instance().m_output_buf, buffer, size, pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS));
}

esp_err_t transport::process_input_int(void* buffer, size_t size) {
	// xStreamBufferReceive can return less than `size` when the stream buffer's
	// trigger level fires earlier (created at level 8). The previous code then
	// bailed with ESP_FAIL, leaving the partial bytes to bleed into the next
	// EVENT_INPUT's batch and desynchronising the firmware->host frame stream.
	// Now we top-up under a single overall deadline so all `size` bytes either
	// arrive together or we fail cleanly with nothing partially written.
	size_t got = 0;
	const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS);
	while (got < size) {
		TickType_t now = xTaskGetTickCount();
		if (now >= deadline) break;
		auto chunk = xStreamBufferReceive(m_input_buf,
		                                  static_cast<uint8_t*>(buffer) + got,
		                                  size - got,
		                                  deadline - now);
		if (chunk == 0) break;
		got += chunk;
	}
	if (got != size) {
		ESP_LOGE(TAG, "Input buffer receive error: got %u expect %u!",
		         unsigned(got), unsigned(size));
		return ESP_FAIL;
	}
	return write_int(buffer, size);
}

esp_err_t transport::send_int(const void* data, size_t size) {
	if (data == NULL) {
        return ESP_FAIL;
    }

    app::ctx_t ncp_event = {
        .event = app::EVENT_INPUT,
        .size = 0
    };

    size_t ret_size = 0;

    {
        // H2: the capacity wait and the StreamBufferSend must be atomic w.r.t.
        // each other. app-task ACKs (send_ack/send_nack via on_rx_packet) race
        // ZBOSS-task send_data; with the space check outside the lock both
        // writers pass the gate and the second short-writes a partial CRC-
        // prefixed frame, desyncing the host stream. Holding m_input_sem across
        // the wait serialises the two writers. The wait stays bounded by
        // RINGBUF_TIMEOUT_MS, and the FIFO drainer is the app task (not parked
        // on this mutex), so no new deadlock is introduced.
        utils::sem_lock l(m_input_sem);

        int count = RINGBUF_TIMEOUT_MS / 10;
        while (xStreamBufferSpacesAvailable(m_input_buf) < size) {
            if (count == 0) break;
            vTaskDelay(pdMS_TO_TICKS(10));
            count--;
        }
        if (count == 0) {
            ESP_LOGE(TAG, "input_buf not enough");
            return ESP_FAIL;
        }

        ret_size = xStreamBufferSend(m_input_buf, data, size, 0);
    }

    if (ret_size != size) {
        ESP_LOGE(TAG, "input_buf send error: size %d expect %d", ret_size, size);
        return ESP_FAIL;
    }
    ncp_event.size = size;
    return app::send_event(ncp_event);
}

esp_err_t transport::init_int() {
	ESP_LOGI(TAG,"init");

    m_input_buf = xStreamBufferCreate(RINGBUF_SIZE, 8);
    if (!m_input_buf) {
        ESP_LOGE(TAG, "Input buffer create error");
        return ESP_ERR_NO_MEM;
    }

    m_output_buf = xStreamBufferCreate(RINGBUF_SIZE, 8);
    if (!m_output_buf) {
        ESP_LOGE(TAG, "Out buffer create error");
        return ESP_ERR_NO_MEM;
    }

    m_input_sem = xSemaphoreCreateMutex();
    m_output_sem = xSemaphoreCreateMutex();
    m_client_sem = xSemaphoreCreateMutex();
    if (!m_input_sem || !m_output_sem || !m_client_sem) {
        ESP_LOGE(TAG, "semaphore create error");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t transport::start_int() {
	if (!m_input_buf) {
		ESP_LOGE(TAG,"need init");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG,"start");
	if (xTaskCreate(&task, "ncp_tcp", TASK_STACK, this, TASK_PRIORITY, NULL) != pdTRUE) {
		return ESP_FAIL;
	}
	return ESP_OK;
}
