#include <stdint.h>

enum
{
	ble_mode_off = 0,
	ble_mode_advertising,
	ble_mode_connection,
	ble_mode_error
};

enum
{
	ble_adv_idle = 0,
	ble_adv_sent,
	ble_adv_scan_request,
	ble_adv_scan_response,
	ble_adv_conn_request,
	ble_adv_conn_ready
};

enum
{
	ble_conn_off = 0,
	ble_conn_init,
	ble_conn_wait,
	ble_conn_ok,
	ble_conn_timeout
};

enum
{
	ble_radio_off = 0,
	ble_radio_rx = 0x01,
	ble_radio_tx = 0x02
};


void ble_init_radio();
void ble_set_our_mac(uint8_t *mac);
//void ble_set_advertiser_channel(uint8_t channel_num); //255 - auto circle
//void ble_send_advertisement(uint8_t *payload, int length);
void ble_radio_irq();

int ble_channel_to_frequency(int channel);

void ble_LL_send_PDU(uint32_t addr, int pdu_length, uint8_t *pdu, int ble_channel);
void ble_LL_respond_PDU(uint32_t addr, int pdu_length, uint8_t *pdu, int ble_channel);
int ble_prepare_adv_pdu(uint8_t *pdu, int payload_len, uint8_t *payload, uint8_t type, uint8_t rand_rx, uint8_t rand_tx);
int ble_prepare_data_pdu(uint8_t *pdu, int payload_len, uint8_t *payload, uint8_t ll_data_header1);

uint32_t ble_get_last_rx_id();
int ble_get_last_rx_pack(uint8_t *buf, int max_buf_len);
int ble_get_last_rx_pack_crc();

void *ble_get_ll_link();
int ble_get_conn_state();

int ble_get_current_MTU();

void ble_peripheral_set_ER(uint8_t *er_text);
void ble_peripheral_set_IR(uint8_t *ir_text);
void ble_peripheral_generate_keys();

void ble_peripheral_generate_mac(uint8_t *res);
int ble_peripheral_in_pairing();
