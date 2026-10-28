#pragma once
#include <cstdint>
#include <atomic>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/stream_buffer.h>
#include "sdkconfig.h"

// The host link: one TCP client at a time. A connection becomes the host in
// three steps, so that no byte or frame of one host's session reaches the
// other's:
//   1. the server task accepts it; while another host is connected it only
//      takes over once it sends a ZBOSS frame (DE AD), so a port scan or a
//      monitoring probe never ends a live session;
//   2. app task, EVENT_LINK_RESET: the link state is reset and the boot
//      frames go to the new socket directly;
//   3. app task, EVENT_LINK_ACTIVE: frames are routed to it. Everything
//      queued between 1 and 3 was stamped for the previous host and is
//      dropped.
class transport {
private:
	static constexpr size_t BUF_SIZE = 1024;
	static constexpr size_t RINGBUF_SIZE = 1024*20;
	static constexpr size_t RINGBUF_TIMEOUT_MS = 50;
	static constexpr size_t TASK_STACK = 1024*4;
	static constexpr size_t TASK_PRIORITY = 18;

	transport();
	static transport& instance();
	esp_err_t init_int();
	esp_err_t start_int();
	StreamBufferHandle_t m_input_buf;                    /*!< The pointer to storage the data from NCP */
    StreamBufferHandle_t m_output_buf;                   /*!< The pointer to storage the data to NCP */
    SemaphoreHandle_t m_input_sem;        /*!< A semaphore handle for process the data from NCP */
    SemaphoreHandle_t m_output_sem;       /*!< Serialises writers of m_output_buf
                                               (FreeRTOS stream buffers are single-writer) */
	// Held by every send to a host socket and by the server task while it
	// closes one, so a frame never goes to a socket being torn down or to the
	// next connection that reuses its fd number.
	SemaphoreHandle_t m_client_sem = nullptr;

	// The socket frames are routed to; -1 while there is none, and frames are
	// then dropped. Atomic so that close_client() can read it without the
	// semaphore, which a stalled send may hold for seconds.
	std::atomic<int> m_client{-1};
	// The socket between steps 1 and 3 above.
	int m_switching = -1;
	// App task only.
	bool m_boot_ready = false;
	int m_listen = -1;

	uint8_t m_temp_buf[BUF_SIZE];

	esp_err_t write_int(const void* data,size_t size);
	void task_int();
	void promote(int fd);
	void drop_fd(int fd);
	void close_client_int();
	void on_link_reset_int(int fd);
	void on_link_active_int(int fd);
	void on_boot_ready_int();
	void send_direct(int fd, const void* data, size_t size);
	void rx_pump(const uint8_t* data, int readed);

	static void task(void *pvParameter) {
		static_cast<transport*>(pvParameter)->task_int();
	}

	esp_err_t send_int(const void* data,size_t size);

	esp_err_t process_input_int(void* buffer,size_t size);
public:
	static esp_err_t init() { return instance().init_int(); }
	static esp_err_t start() { return instance().start_int(); }

	static esp_err_t send(const void* data,size_t size) { return instance().send_int(data,size); }

	static esp_err_t process_input(void* buffer,size_t size) { return instance().process_input_int(buffer,size); }
	static size_t output_receive(void* buffer,size_t size);

	// App task, from the events named above.
	static void on_link_reset(int fd) { instance().on_link_reset_int(fd); }
	static void on_link_active(int fd) { instance().on_link_active_int(fd); }
	static void on_boot_ready() { instance().on_boot_ready_int(); }

	// End the host's connection with a FIN. The NCP's own restarts go
	// through restart() so the host sees the link end at once instead of
	// waiting out its keepalive; a restart espOS makes (an OTA, the web UI)
	// does not, and the host notices within the keepalive's ~24 s.
	static void close_client() { instance().close_client_int(); }
	[[noreturn]] static void restart();
};
