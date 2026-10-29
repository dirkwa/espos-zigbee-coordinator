#pragma once
#include <atomic>
#include <cstdint>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class protocol {
private:
	struct ncp_header_t {
		uint8_t signature[2];
		uint16_t packet_len;
		uint8_t packet_type;
		uint8_t is_ack: 1;
		// On the wire this bit is the spec's RETRANSMIT flag. Its meaning
		// depends on is_ack: ACK+retransmit = NACK ("resend frame ack_seq");
		// DATA+retransmit = "this data frame is a retransmission" (host did
		// not see our ACK, or the original frame was lost). The legacy field
		// name is_nack only describes the first case — see DUP-1 in
		// protocol.cpp for the data-frame case.
		uint8_t is_nack: 1;
		uint8_t packet_seq: 2;
		uint8_t ack_seq: 2;
		uint8_t first_fragment: 1;
		uint8_t last_fragment: 1;
		uint8_t header_crc;
	} __attribute__((packed));
	static_assert(sizeof(ncp_header_t)==7);

	protocol();
	static protocol& instance();
	esp_err_t init_int();
	esp_err_t start_int();

	static constexpr size_t RX_BUFFER_SIZE = 1024;
	// 2048 covers worst-case ZDO responses with full 64-entry tables:
	// ZDO_MGMT_LQI_REQ -> 64*sizeof(zb_zdo_neighbor_table_record_t) ~ 1.4 KB,
	// ZDO_MGMT_BIND_REQ -> 64*sizeof(zb_zdo_binding_table_record_t) ~ 1.5 KB.
	// Pre-fix size 256 silently rejected anything larger via send_data_int's
	// "too long" guard, so the host never saw populated neighbor / binding
	// tables once the network grew past ~10 nodes.
	static constexpr size_t TX_BUFFER_SIZE = 2048;
	static constexpr uint8_t ZBOSS_NCP_API_HL = 0x06;

	uint8_t m_rx_buffer[RX_BUFFER_SIZE];
	size_t m_rx_buffer_pos;
	uint8_t m_tx_seq;
	// DUP-1 duplicate-drop state: packet_seq of the last ACCEPTED (ACKed +
	// dispatched) host data frame. Only touched on the app task (single
	// consumer of on_rx_packet) — no locking needed. m_last_rx_seq_valid
	// gates the very first frame after boot (any seq is fresh then).
	uint8_t m_last_rx_seq;
	bool m_last_rx_seq_valid;

	uint8_t m_tx_buffer[TX_BUFFER_SIZE];
	SemaphoreHandle_t m_tx_sem;        /*!< A semaphore handle send_data, becouse it use buffer */
	// Counts link resets, changed only with m_tx_sem held. Never 0, which
	// reply_scope reserves for "not a reply".
	std::atomic<uint32_t> m_epoch{1};
	static thread_local uint32_t t_reply_epoch;

	esp_err_t on_rx_int(const void* data,size_t size);
	void on_rx_packet(const ncp_header_t& hdr,const void* data,size_t data_size);
	void send_ack(const ncp_header_t& hdr);
	void send_nack(const ncp_header_t& hdr);
	esp_err_t send_data_int(const void* data,size_t size);
	void reset_link_state_locked();
	void lock_tx();
	void unlock_tx();
	
public:
	// Largest single frame the protocol layer ever hands to transport::send
	// (NCP->host direction = m_tx_buffer). app::m_buffer must be >= this or an
	// oversize frame (e.g. a full 64-entry Mgmt_Lqi/Mgmt_Bind table) strands in
	// transport's m_input_buf and permanently desyncs the link — see the
	// static_assert in app::process_event (H1).
	static constexpr size_t MAX_FRAME_SIZE = TX_BUFFER_SIZE;

	static esp_err_t init() { return instance().init_int(); }
	static esp_err_t start() { return instance().start_int(); }

	static esp_err_t on_rx(const void* data,size_t size) {
		return instance().on_rx_int(data,size);
	}
	static esp_err_t send_data(const void* data,size_t size) {
		return instance().send_data_int(data,size);
	}
	// The link a request arrived on, read on the app task while it
	// processes the request.
	static uint32_t epoch() {
		return instance().m_epoch.load(std::memory_order_relaxed);
	}
	// A reply sent later, from another task, is for the host whose request
	// it answers. While a scope is open, send_data drops frames on this
	// task once the link has been reset since `epoch`: a new host would
	// take the reply's tsn for one of its own requests.
	class reply_scope {
	public:
		explicit reply_scope(uint32_t epoch) : m_prev(t_reply_epoch) { t_reply_epoch = epoch; }
		~reply_scope() { t_reply_epoch = m_prev; }
		reply_scope(const reply_scope&) = delete;
		reply_scope& operator=(const reply_scope&) = delete;
	private:
		uint32_t m_prev;
	};
	// Back to the state after a boot, for a host that has just connected:
	// its sequence numbers restart at 0 and it has seen none of ours. App
	// task only, like on_rx. `cut` runs first with m_tx_sem held, so no frame
	// is stamped while it runs: everything transport holds at that point was
	// stamped for the previous host. The reset happens only when it returns
	// true.
	template <typename F>
	static void reset_link_state(F&& cut) {
		protocol& p = instance();
		p.lock_tx();
		if (cut()) {
			p.reset_link_state_locked();
		}
		p.unlock_tx();
	}
};