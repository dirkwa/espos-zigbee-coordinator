#include "transport.h"
#include "app.h"
#include "protocol.h"
#include "utils.h"
#include "zb_ncp.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <algorithm>
#include <cerrno>
#include <cstdint>
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

// A candidate must send a ZBOSS frame within this long of the boot-ready
// frame to take over; until the stack is up a host has nothing to answer.
static constexpr int64_t QUALIFY_TIMEOUT_US = 10LL * 1000 * 1000;
// And a connection waits at most this long for a stack that is not up.
static constexpr int64_t PENDING_TIMEOUT_US = 60LL * 1000 * 1000;

// The framing-level ACK herdsman's open-port handshake waits for
// (andryblack/esp-coordinator#11). A host connects whenever it likes and may
// reconnect, so every host gets it, not only the first.
static const uint8_t BOOT_ACK[] = {0xDE, 0xAD, 0x05, 0x00, 0x06, 0x01, 0x8F};

transport::transport() {
	for (auto& fd : m_cands) {
		fd.store(-1);
	}
}

transport& transport::instance() {
	static transport s_transport;
	return s_transport;
}

bool transport::send_all(int fd, const void* data, size_t size)
{
    const auto* p = static_cast<const uint8_t*>(data);
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
        return true;
    }
    // A frame cut short leaves the host's parser mid-frame, so the link
    // cannot carry on: end it, and the host reconnects to fresh boot frames.
    ESP_LOGE(TAG, "send stalled at %u/%u bytes (errno %d), closing the host link",
             unsigned(off), unsigned(size), errno);
    ::shutdown(fd, SHUT_RDWR);
    return false;
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
    return send_all(fd, buffer, size) ? ESP_OK : ESP_FAIL;
}

// A full second of a full queue means the app task is stuck.
static constexpr int64_t EVENT_POST_DEADLINE_US = 1000LL * 1000;

bool transport::rx_pump(const uint8_t* data, int readed) {
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
        // protocol::on_rx takes at most its RX_BUFFER_SIZE per call, and a
        // promotion pumps a read plus the scanned tail before it.
        const size_t chunk = std::min<size_t>(readed - off, BUF_SIZE);
        size_t sent = xStreamBufferSend(m_output_buf, data + off,
                                        chunk,
                                        pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS));
        if (sent == 0) {
            ESP_LOGE(TAG, "output_buf full, dropping %d bytes", readed - off);
            return false;
        }
        ncp_event.size = sent;
        // Bounded wait on a momentarily-full event queue: the bytes are
        // already committed to m_output_buf, so a dropped event would orphan
        // them and shift the byte/event pairing. Waiting HERE is safe —
        // rx_pump only ever runs on the server task, never on the ZBOSS task
        // (app::send_event itself must stay non-blocking for that caller —
        // see the critical-section panic note there).
        const int64_t deadline = esp_timer_get_time() + EVENT_POST_DEADLINE_US;
        while (app::post_event(ncp_event, pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS)) != ESP_OK) {
            if (esp_timer_get_time() >= deadline) {
                ESP_LOGE(TAG, "event queue full, %u host bytes orphaned", unsigned(sent));
                m_orphaned.fetch_add(sent);
                return false;
            }
        }
        off += sent;
    }
    return true;
}

void transport::close_client_int() {
    // No semaphore: a stalled send can hold it for seconds, and this runs on
    // the restart path, sometimes from the esp_timer task. shutdown is safe
    // beside a send on the same socket; only the server task closes one.
    for (int fd : {m_client.load(), m_switching.load()}) {
        if (fd >= 0) {
            ::shutdown(fd, SHUT_RDWR);
        }
    }
    for (auto& c : m_cands) {
        const int fd = c.load();
        if (fd >= 0) {
            ::shutdown(fd, SHUT_RDWR);
        }
    }
}

void transport::drop_fd(int fd) {
    if (fd < 0) {
        return;
    }
    utils::sem_lock l(m_client_sem);
    if (m_client.load(std::memory_order_relaxed) == fd) {
        m_client.store(-1, std::memory_order_relaxed);
    }
    if (m_switching.load(std::memory_order_relaxed) == fd) {
        m_switching.store(-1, std::memory_order_relaxed);
        m_switching_gen = 0;
    }
    for (auto& c : m_cands) {
        if (c.load(std::memory_order_relaxed) == fd) {
            c.store(-1, std::memory_order_relaxed);
        }
    }
    ::close(fd);
}

bool transport::add_candidate(int slot, int fd) {
    utils::sem_lock l(m_client_sem);
    m_cands[slot].store(fd, std::memory_order_relaxed);
    // The boot frames go out on accept: a host waits for them before it
    // sends anything, and it is its first frame that makes it the host.
    // Under the semaphore with the m_boot_ready check, so the boot-ready
    // frame reaches it exactly once whether the stack comes up before or
    // after this.
    if (!send_all(fd, BOOT_ACK, sizeof(BOOT_ACK)) || !m_boot_ready.load()) {
        return false;
    }
    size_t n = 0;
    const uint8_t* frame = zb_ncp::boot_ready_frame(&n);
    return send_all(fd, frame, n);
}

// The app task can be several frames into stalled sends to the previous host.
static constexpr uint32_t LINK_RESET_WAIT_MS = 15000;

bool transport::promote(int fd) {
    uint16_t gen;
    {
        utils::sem_lock l(m_client_sem);
        if (++m_gen == 0) ++m_gen;
        gen = m_gen;
        for (auto& c : m_cands) {
            if (c.load(std::memory_order_relaxed) == fd) {
                c.store(-1, std::memory_order_relaxed);
            }
        }
        m_switching.store(fd, std::memory_order_relaxed);
        m_switching_gen = gen;
    }
    // The generation, not the fd, rides in the event: lwIP reuses a closed
    // fd number at once, so an event for a host already gone would match the
    // next connection. Only the server task posts these, never the app task
    // that drains the queue, so waiting for room cannot deadlock.
    app::ctx_t ev = {.event = app::EVENT_LINK_RESET, .size = gen};
    xQueueReset(m_reset_done);
    app::post_event(ev, portMAX_DELAY);
    // The host's bytes go in only once its reset has run: before it, they
    // would sit among the previous host's in m_output_buf, where the reset
    // could not tell the two apart.
    // Elapsed ticks, not a deadline, so a tick count wrap cannot end the
    // wait early.
    const TickType_t start = xTaskGetTickCount();
    const TickType_t wait = pdMS_TO_TICKS(LINK_RESET_WAIT_MS);
    uint32_t done = 0;
    while (true) {
        const TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= wait ||
            xQueueReceive(m_reset_done, &done, wait - elapsed) != pdTRUE) {
            ESP_LOGE(TAG, "link reset did not run, dropping the new host");
            return false;
        }
        if ((done & 0xFFFF) == gen) {
            break;
        }
    }
    if ((done >> 16) == 0) {
        return false;
    }
    ESP_LOGI(TAG, "host connected");
    return true;
}

void transport::discard_orphaned() {
    // Every event queued before this one has claimed its bytes, and the
    // host that orphaned these was dropped before any later host's bytes
    // were pumped, so they are the oldest in the buffer. A count raised
    // after this ran waits for the next host's reset, with nothing written
    // in between.
    uint8_t sink[64];
    size_t left = m_orphaned.load();
    while (left > 0) {
        size_t n = xStreamBufferReceive(m_output_buf, sink, std::min(left, sizeof(sink)), 0);
        if (n == 0) {
            break;
        }
        left -= n;
        m_orphaned.fetch_sub(n);
    }
}

void transport::on_link_reset_int(uint16_t gen) {
    discard_orphaned();
    bool ok = false;
    protocol::reset_link_state([&]() {
        utils::sem_lock l(m_client_sem);
        if (gen != m_switching_gen) {
            return false;   // gone or replaced before its turn
        }
        // No frame can be stamped while protocol holds m_tx_sem, so what
        // m_input_buf holds now is exactly what was stamped for the previous
        // host, whose sequence numbers would confuse the new one. Nobody
        // blocks on the buffer (every send and receive polls), as a reset
        // requires.
        {
            utils::sem_lock in(m_input_sem);
            if (xMessageBufferReset(m_input_buf) != pdPASS) {
                ESP_LOGE(TAG, "input_buf reset failed, the new host may see old frames");
            }
        }
        m_client.store(m_switching.load(std::memory_order_relaxed), std::memory_order_relaxed);
        m_switching.store(-1, std::memory_order_relaxed);
        m_switching_gen = 0;
        ok = true;
        return true;
    });
    const uint32_t done = uint32_t(gen) | (uint32_t(ok) << 16);
    xQueueOverwrite(m_reset_done, &done);
}

void transport::poll_boot_ready_int(void* buffer, size_t size) {
    if (!m_boot_ready_req.load(std::memory_order_relaxed) || m_boot_ready.load()) {
        return;
    }
    // A host already routed gets the frame after what was queued for it
    // before now, so send that first. App task, which is the one draining
    // the queue.
    process_input_int(buffer, size, SIZE_MAX);
    utils::sem_lock l(m_client_sem);
    size_t n = 0;
    const uint8_t* frame = zb_ncp::boot_ready_frame(&n);
    // Whichever slot holds a connection has had BOOT_ACK and not this.
    for (int fd : {m_client.load(), m_switching.load()}) {
        if (fd >= 0) {
            send_all(fd, frame, n);
        }
    }
    for (auto& c : m_cands) {
        const int fd = c.load(std::memory_order_relaxed);
        if (fd >= 0) {
            send_all(fd, frame, n);
        }
    }
    // Only now: the server task starts a candidate's timeout when it sees
    // this, and the sends above can take seconds.
    m_boot_ready.store(true);
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

// Index of the first valid ZBOSS frame header in buf, -1 when there is none.
static int find_header(const uint8_t* buf, int len) {
    for (int i = 0; i + transport::HEADER_SIZE <= len; ++i) {
        if (buf[i] == 0xDE && buf[i + 1] == 0xAD &&
            utils::crc8(buf + i + 2, 4) == buf[i + transport::HEADER_SIZE - 1]) {
            return i;
        }
    }
    return -1;
}

// recv that never blocks: select's readiness bits can be stale for a socket
// closed and reused within the same pass. 0 = nothing to read now, -1 = the
// connection ended.
static int recv_now(int fd, uint8_t* buf, size_t size) {
    int n = ::recv(fd, buf, size, MSG_DONTWAIT);
    if (n > 0) {
        return n;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
        return 0;
    }
    return -1;
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

    struct cand_t {
        int fd = -1;
        int64_t accepted = 0;
        int64_t since = 0;      // when it got the boot-ready frame; 0 = not yet
        uint8_t tail[HEADER_SIZE - 1];
        int tail_len = 0;
    };
    int host = -1;              // promoted: switching, then routed
    cand_t cands[MAX_CANDIDATES];
    while (true) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(m_listen, &rfds);
        int maxfd = m_listen;
        auto watch = [&](int fd) {
            if (fd >= 0) {
                FD_SET(fd, &rfds);
                if (fd > maxfd) maxfd = fd;
            }
        };
        watch(host);
        for (const auto& c : cands) {
            watch(c.fd);
        }
        struct timeval tv = {};
        tv.tv_sec = 1;
        if (::select(maxfd + 1, &rfds, nullptr, nullptr, &tv) < 0) {
            if (errno != EINTR) vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (host >= 0 && FD_ISSET(host, &rfds)) {
            int n = recv_now(host, m_temp_buf, BUF_SIZE);
            // A host whose bytes were lost mid-stream cannot carry on: end
            // it, and it reconnects to fresh boot frames.
            if (n > 0 && !rx_pump(m_temp_buf, n)) {
                ESP_LOGW(TAG, "host bytes lost, closing the host link");
                drop_fd(host);
                host = -1;
            } else if (n < 0) {
                ESP_LOGW(TAG, "host disconnected");
                drop_fd(host);
                host = -1;
            }
        }

        const int64_t now = esp_timer_get_time();
        for (auto& c : cands) {
            if (c.fd < 0) {
                continue;
            }
            if (FD_ISSET(c.fd, &rfds)) {
                int n = recv_now(c.fd, m_temp_buf, BUF_SIZE);
                if (n < 0) {
                    drop_fd(c.fd);
                    c.fd = -1;
                    continue;
                }
                if (n > 0) {
                    memcpy(m_scan_buf, c.tail, c.tail_len);
                    memcpy(m_scan_buf + c.tail_len, m_temp_buf, n);
                    const int len = c.tail_len + n;
                    const int at = find_header(m_scan_buf, len);
                    if (at < 0) {
                        // A header that began here ends in the next read.
                        c.tail_len = len < HEADER_SIZE - 1 ? len : HEADER_SIZE - 1;
                        memcpy(c.tail, m_scan_buf + len - c.tail_len, c.tail_len);
                    } else {
                        if (host >= 0) {
                            ESP_LOGW(TAG, "new host replaces the connected one");
                            drop_fd(host);
                        }
                        host = c.fd;
                        c.fd = -1;
                        if (!promote(host)) {
                            drop_fd(host);
                            host = -1;
                        } else if (!rx_pump(m_scan_buf + at, len - at)) {
                            ESP_LOGW(TAG, "host bytes lost, closing the host link");
                            drop_fd(host);
                            host = -1;
                        }
                        continue;
                    }
                }
            }
            if (c.since == 0 && m_boot_ready.load()) {
                c.since = now;
            }
            if ((c.since != 0 && now - c.since > QUALIFY_TIMEOUT_US) ||
                (c.since == 0 && now - c.accepted > PENDING_TIMEOUT_US)) {
                ESP_LOGW(TAG, "connection sent no ZBOSS frame in %llds, dropped",
                         QUALIFY_TIMEOUT_US / 1000000);
                drop_fd(c.fd);
                c.fd = -1;
            }
        }

        if (FD_ISSET(m_listen, &rfds)) {
            int fd = ::accept(m_listen, nullptr, nullptr);
            if (fd >= 0) {
                set_client_options(fd);
                int i = 0;
                for (int j = 0; j < MAX_CANDIDATES; ++j) {
                    if (cands[j].fd < 0) {
                        i = j;
                        break;
                    }
                    if (cands[j].accepted < cands[i].accepted) {
                        i = j;
                    }
                }
                cand_t* slot = &cands[i];
                if (slot->fd >= 0) {
                    ESP_LOGW(TAG, "too many connections waiting, the oldest dropped");
                    drop_fd(slot->fd);
                }
                *slot = cand_t{};
                slot->fd = fd;
                slot->accepted = esp_timer_get_time();
                if (add_candidate(i, fd)) {
                    slot->since = slot->accepted;
                }
            }
        }
    }
}

size_t transport::output_receive(void* buffer, size_t size) {
	return xStreamBufferReceive(instance().m_output_buf, buffer, size, pdMS_TO_TICKS(RINGBUF_TIMEOUT_MS));
}

void transport::process_input_int(void* buffer, size_t size, size_t max_frames) {
	size_t n;
	while (max_frames-- > 0 && (n = xMessageBufferReceive(m_input_buf, buffer, size, 0)) > 0) {
		write_int(buffer, n);
	}
}

esp_err_t transport::send_int(const void* data, size_t size) {
	if (data == NULL) {
        return ESP_FAIL;
    }
	// A frame larger than the drain buffer would block the queue for good.
	if (size > protocol::MAX_FRAME_SIZE) {
        ESP_LOGE(TAG, "frame of %u bytes is too large, dropped", unsigned(size));
        return ESP_FAIL;
    }

    app::ctx_t ncp_event = {
        .event = app::EVENT_INPUT,
        .size = 0
    };

    size_t ret_size = 0;

    {
        // H2: app-task ACKs (send_ack/send_nack via on_rx_packet) race
        // ZBOSS-task send_data, and FreeRTOS message buffers are
        // single-writer. Holding m_input_sem across the wait serialises the
        // two writers. The wait stays bounded by
        // RINGBUF_TIMEOUT_MS, and the FIFO drainer is the app task (not parked
        // on this mutex), so no new deadlock is introduced.
        utils::sem_lock l(m_input_sem);

        int count = RINGBUF_TIMEOUT_MS / 10;
        while ((ret_size = xMessageBufferSend(m_input_buf, data, size, 0)) == 0 && count-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (ret_size != size) {
        ESP_LOGE(TAG, "input_buf not enough for %u bytes", unsigned(size));
        return ESP_FAIL;
    }
    ncp_event.size = size;
    // Queued either way: without the event the next drain sends it, so a
    // full queue is not worth a log line from the ZBOSS task.
    app::post_event(ncp_event, 0);
    return ESP_OK;
}

esp_err_t transport::init_int() {
	ESP_LOGI(TAG,"init");

    m_input_buf = xMessageBufferCreate(RINGBUF_SIZE);
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
    m_reset_done = xQueueCreate(1, sizeof(uint32_t));
    if (!m_input_sem || !m_output_sem || !m_client_sem || !m_reset_done) {
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
