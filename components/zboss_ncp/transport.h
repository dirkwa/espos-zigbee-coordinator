#pragma once
#include <cstdint>
#include <atomic>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/message_buffer.h>
#include <freertos/stream_buffer.h>
#include "sdkconfig.h"

// The host link: one TCP client at a time, so that no byte or frame of one
// host's session reaches another's.
//   1. Every connection starts as a candidate and is sent the boot frames.
//      It becomes the host once it sends a ZBOSS frame header with a valid
//      CRC, so a port scan or a monitoring probe never ends a live session,
//      and a host reconnecting from a half-open session takes over at once.
//   2. The server task then puts it in the switching slot and posts
//      EVENT_LINK_RESET with its generation, ahead of the bytes it sent.
//   3. The app task resets the link state and, in the same step, switches
//      the route to it and drops every frame still queued, all of them
//      stamped for the previous host.
// A response the stack produces after the switch to a request of the
// previous host goes to the new one, which has no request it could match.
class transport {
public:
	// A ZBOSS frame header: DE AD, length, type, flags, CRC-8 of the four
	// bytes before it.
	static constexpr int HEADER_SIZE = 7;

private:
	static constexpr size_t BUF_SIZE = 1024;   // == protocol::RX_BUFFER_SIZE
	static constexpr size_t RINGBUF_SIZE = 1024*20;
	static constexpr size_t RINGBUF_TIMEOUT_MS = 50;
	static constexpr size_t TASK_STACK = 1024*4;
	static constexpr size_t TASK_PRIORITY = 18;

	transport();
	static transport& instance();
	esp_err_t init_int();
	esp_err_t start_int();
	static constexpr int MAX_CANDIDATES = 3;

	// One whole frame per message, so a frame whose event was dropped on a
	// full queue is sent by the next drain instead of shifting the stream.
	MessageBufferHandle_t m_input_buf;                   /*!< The pointer to storage the data from NCP */
    StreamBufferHandle_t m_output_buf;                   /*!< The pointer to storage the data to NCP */
    SemaphoreHandle_t m_input_sem;        /*!< A semaphore handle for process the data from NCP */
    SemaphoreHandle_t m_output_sem;       /*!< Serialises writers of m_output_buf
                                               (FreeRTOS stream buffers are single-writer) */
	// Held by every send to a host socket and while one is closed or moves
	// between the slots below, so a frame never goes to a socket being torn
	// down or to the next connection that reuses its fd number.
	SemaphoreHandle_t m_client_sem = nullptr;

	// The slots of the steps above. Atomic so that close_client() can read
	// them without the semaphore, which a stalled send may hold.
	std::atomic<int> m_client{-1};      // frames are routed here; -1 drops them
	std::atomic<int> m_switching{-1};   // promoted, its EVENT_LINK_RESET queued
	// Several, so that a probe connecting just before a reconnecting host
	// does not cost the host its slot.
	std::atomic<int> m_cands[MAX_CANDIDATES];
	uint16_t m_switching_gen = 0;
	uint16_t m_gen = 0;

	// Set under m_client_sem: whether the boot-ready frame has gone out, so
	// that every connection after it is sent it too, exactly once. The server
	// task reads it without the semaphore to start a candidate's timeout.
	std::atomic<bool> m_boot_ready{false};
	std::atomic<bool> m_boot_ready_req{false};

	// Host bytes in m_output_buf whose EVENT_OUTPUT could not be queued.
	// The app task drops them before the next host's link reset.
	std::atomic<size_t> m_orphaned{0};
	void discard_orphaned();

	int m_listen = -1;

	uint8_t m_temp_buf[BUF_SIZE];
	// A candidate's unscanned tail followed by its latest read.
	uint8_t m_scan_buf[HEADER_SIZE - 1 + BUF_SIZE];

	esp_err_t write_int(const void* data,size_t size);
	bool send_all(int fd, const void* data, size_t size);
	void task_int();
	// `slot` is the server task's index for it, mirrored in m_cands.
	bool add_candidate(int slot, int fd);
	void promote(int fd);
	void drop_fd(int fd);
	void close_client_int();
	void on_link_reset_int(uint16_t gen);
	void poll_boot_ready_int(void* buffer, size_t size);
	// False when host bytes were lost: the link must end.
	bool rx_pump(const uint8_t* data, int readed);

	static void task(void *pvParameter) {
		static_cast<transport*>(pvParameter)->task_int();
	}

	esp_err_t send_int(const void* data,size_t size);

	void process_input_int(void* buffer,size_t size,size_t max_frames);
public:
	static esp_err_t init() { return instance().init_int(); }
	static esp_err_t start() { return instance().start_int(); }

	static esp_err_t send(const void* data,size_t size) { return instance().send_int(data,size); }

	// App task: send up to `max_frames` queued frames to the host, oldest
	// first. `size` is the buffer's, at least protocol::MAX_FRAME_SIZE.
	// Frames outnumber their events only when an event was dropped on a full
	// queue, so each event takes one extra and the app loop the rest once
	// the queue is empty; an unbounded drain could keep the app task from
	// the host's requests while Wi-Fi is slow.
	static void process_input(void* buffer,size_t size,size_t max_frames) { instance().process_input_int(buffer,size,max_frames); }
	static size_t output_receive(void* buffer,size_t size);

	// App task, EVENT_LINK_RESET.
	static void on_link_reset(uint16_t gen) { instance().on_link_reset_int(gen); }
	// Any task: the stack has loaded its NVRAM. The frame goes out from the
	// app task, which also polls for it in case the event that carries the
	// request was dropped on a full queue.
	static void request_boot_ready() { instance().m_boot_ready_req.store(true); }
	// `buffer` as for process_input.
	static void poll_boot_ready(void* buffer, size_t size) { instance().poll_boot_ready_int(buffer, size); }

	// End the host's connection with a FIN. The NCP's own restarts go
	// through restart() so the host sees the link end at once instead of
	// waiting out its keepalive; a restart espOS makes (an OTA, the web UI)
	// does not, and the host notices within the keepalive's ~24 s.
	static void close_client() { instance().close_client_int(); }
	[[noreturn]] static void restart();
};
