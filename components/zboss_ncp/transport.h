#pragma once
#include <cstdint>
#include <atomic>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/stream_buffer.h>
#include "sdkconfig.h"

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
	// Held while a send uses m_client and while the server task closes or
	// replaces it, so a frame never goes to a socket that is being torn down
	// or to the next client that reuses the fd number.
	SemaphoreHandle_t m_client_sem;

	// The host's socket, -1 while no host is connected. Frames for a missing
	// host are dropped: a host that connects later starts from the boot frames
	// the server sends it, not from a backlog.
	int m_client = -1;
	// Accepted, not yet switched in by on_link_reset().
	int m_pending = -1;
	int m_listen = -1;

	uint8_t m_temp_buf[BUF_SIZE];

	esp_err_t write_int(const void* data,size_t size);
	void task_int();
	void serve_client(int fd);
	void close_client_int();
	void drop_fd(int fd);
	void on_link_reset_int();
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

	// Close the host's connection with a FIN. Every restart goes through
	// restart() below so that the host sees the link end at once instead of
	// waiting out its keepalive on a connection the reboot silently dropped.
	static void close_client() { instance().close_client_int(); }
	// App task: switch to the host accepted last (app::EVENT_LINK_RESET).
	static void on_link_reset() { instance().on_link_reset_int(); }
	[[noreturn]] static void restart();
};
