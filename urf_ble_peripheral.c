#include "urf_ble_peripheral.h"
#include "ble_const.h"
#include "urf_radio.h"
#include "urf_timer.h"
#include "urf_uart.h"
#include "nrf.h"

#include "urf_ble_smp_process.h"
#include "urf_ble_encryption.h"
#include "urf_ble_att_process.h"

#include <stdio.h>

volatile uint8_t ble_tx_buffer[270];
uint8_t ble_tx_buffer_pre[270];
volatile uint8_t ble_rx_buffer[270];

volatile uint8_t ble_tx_buffer_encr[270];
volatile uint8_t ble_rx_buffer_encr[270];
uint8_t ble_encr_temp_buf[280];

uint8_t incoming_mac[6];

uint8_t ble_last_rx_pack_buf[270];
uint8_t *ble_last_rx_pack = ble_last_rx_pack_buf;
uint32_t ble_last_rx_pack_id = 0;
int ble_last_rx_pack_len = 0;
int ble_last_rx_pack_crc = 0;

sLLData ll_link;
sLLData ll_link_update;

int ble_ll_same_event_response = 0; //experimentally found that with iPhones, we must respond
//within the same connection event, with Samsung we _must not_ respond within the same connection
//event, with other manufacturers sometimes both work, sometimes not. 
//0 looks to be a better a priori guess - but if connection was lost without getting to ATT stage,
//we will switch the mode

sATT_link att_link;
sSMP_link smp_link;

int ble_is_connectable = 1;
int ble_conn_mode = ble_mode_off;
int ble_conn_state = ble_adv_idle;
int ble_radio_rxtx = ble_radio_off;
int ble_current_channel = 37;

int ble_channel_to_frequency(int channel)
{
	if(channel == 37) return 2;
	if(channel == 38) return 26;
	if(channel == 39) return 80;
	int freq_shift = 4;
	if(channel > 10) freq_shift = 6;
	return channel*2 + freq_shift;
}

uint8_t ble_LL_channel_map[40];
uint8_t ble_LL_channel_count = 37;

void ble_LL_update_channel_map(sLLData *link)
{
	ble_LL_channel_count = 0;
	for(int b = 0; b < 5; b++)
		for(int bit = 0; bit < 8; bit++)
			if(link->chan_map[b] & (1<<bit))
			{
				ble_LL_channel_map[ble_LL_channel_count] = b*8 + bit;
				ble_LL_channel_count++;
			}
}

uint8_t ble_LL_is_channel_used(sLLData *link, uint8_t chan)
{
	if(link->chan_cnt == 37) return (chan < 37);
	if(chan >= 40) return 0;
	uint8_t id = chan/8;
	uint8_t bit = chan%8;
	return (link->chan_map[id]&(1<<bit)) > 0;
}

uint8_t ble_LL_get_next_channel(sLLData *link)
{
	link->last_channel = (link->last_channel + link->hop)%37;
	if(!ble_LL_is_channel_used(link, link->last_channel))
	{
		return ble_LL_channel_map[link->last_channel%ble_LL_channel_count];
	}
	return link->last_channel;
}

void ble_LL_send_PDU(uint32_t addr, int pdu_length, uint8_t *pdu, int ble_channel)
{
	NRF_RADIO->BASE0 = addr<<8;
	NRF_RADIO->PREFIX0 = addr>>24;
	int freq = ble_channel_to_frequency(ble_channel);
	NRF_RADIO->FREQUENCY = freq;
	NRF_RADIO->DATAWHITEIV = (1<<6) | ble_channel; //bit 6 is hardwired to 1 on nRF52832 but it could be not the case on other chips
	NRF_RADIO->CRCPOLY = 0x00065B;
	NRF_RADIO->CRCINIT = 0x555555;
	
	for(int x = 0; x < pdu_length; x++)
	{
		ble_tx_buffer_pre[x] = ble_tx_buffer[x];
		ble_tx_buffer[x] = pdu[x];
	}
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;
	rf_mode_tx_only();
	ble_radio_rxtx = ble_radio_tx;
}

void ble_LL_respond_PDU(uint32_t addr, int pdu_length, uint8_t *pdu, int ble_channel)
{
	for(int x = 0; x < pdu_length; x++)
	{
		ble_tx_buffer_pre[x] = ble_tx_buffer[x];
		ble_tx_buffer[x] = pdu[x];
	}
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;
	NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;

	ble_radio_rxtx = ble_radio_tx;
}
void ble_send_advertisement_packet(int pdu_length, uint8_t *pdu, int ble_channel)
{
	ble_LL_send_PDU(0x8E89BED6, pdu_length, pdu, ble_channel);
}


void ble_set_connection_mode(int is_connectable)
{
	ble_is_connectable = is_connectable;
}

void ble_set_our_mac(uint8_t *mac)
{
	for(int x = 0; x < 6; x++)
		ll_link.our_mac[x] = mac[x];
}

int ble_add_field_to_pdu(uint8_t *pdu, int pdu_len, uint8_t *data, int data_len, uint8_t data_type)
{
	int pdu_pos = pdu_len;
	pdu[pdu_pos++] = data_len + 1;
	pdu[pdu_pos++] = data_type;
	for(int x = 0; x < data_len; x++)
		pdu[pdu_pos++] = data[x];
	return pdu_pos;
}

int ble_prepare_adv_pdu(uint8_t *pdu, int payload_len, uint8_t *payload, uint8_t type, uint8_t rand_rx, uint8_t rand_tx)
{
	ble_adv_header1 hdr;
	hdr.header = 0;
	hdr.type = type;
	hdr.tx_addr = rand_tx;
	hdr.rx_addr = rand_rx;
	pdu[0] = hdr.header;
	pdu[1] = payload_len; //including header
	pdu[2] = 0; //useless S1 field
	for(int x = 0; x < payload_len; x++)
		pdu[3+x] = payload[x];
	return payload_len+3;
}
int ble_prepare_data_pdu(uint8_t *pdu, int payload_len, uint8_t *payload, uint8_t ll_data_header1)
{
	pdu[0] = ll_data_header1;
	pdu[1] = payload_len;
	pdu[2] = 0; //useless S1 field
	for(int x = 0; x < payload_len; x++)
		pdu[3+x] = payload[x];
	return payload_len+3;
}

void ble_init_radio()
{
	NVIC_DisableIRQ(RADIO_IRQn);
	
	//start random numbers generator - values will be used in random number generator in multiple places
	ble_rand_init();
	rf_dettach_rx_irq();
	rf_dettach_tx_irq();
	rf_override_irq(ble_radio_irq);
	ble_encr_ccm_config(0); //turn off encryption by default
	NRF_RADIO->POWER = 1;
	NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos;
	
	NRF52_PCNF0_REG conf0;
	conf0.reg = 0;
	conf0.length_bitsz = 8;
	conf0.S0_bytesz = 1;
	conf0.S1_bitsz = 0; //NEED TO CHECK
	conf0.S1_include = 1; //NRF encryption hardware requires this field - so all other processing
	//will have to add an empty byte all for nothing. Thanks, nordic...
	conf0.preamble_length = 0;
	
    // Packet configuration
    NRF_RADIO->PCNF0 = conf0.reg;
	
	NRF52_PCNF1_REG conf1;
	conf1.reg = 0;
	conf1.max_length = 250;
	conf1.added_length = 0; 
	conf1.addr_len = 3;
	conf1.endian = 0; //need to confirm
	conf1.whitening = 1;
	
    // Packet configuration
    NRF_RADIO->PCNF1 = conf1.reg;
	
	NRF_RADIO->BASE0   = 0x89BED600;
	NRF_RADIO->PREFIX0 = 0x8E;// << RADIO_PREFIX0_AP0_Pos;

	// Use logical address 0 (BASE0 + PREFIX0 byte 0)
	NRF_RADIO->TXADDRESS = 0;// << RADIO_TXADDRESS_TXADDRESS_Pos;
	NRF_RADIO->RXADDRESSES = 1;// << RADIO_TXADDRESS_TXADDRESS_Pos;
  
	NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
			(RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);

	NRF_RADIO->TIFS = 150;
	
	NRF_RADIO->CRCPOLY = 0x00065B;
	NRF_RADIO->CRCINIT = 0x555555;
  
	NRF_RADIO->TXPOWER = 0x04;// 0x04 max RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0; 
//	NRF_RADIO->EVENTS_READY = 0;
//	NRF_RADIO->EVENTS_ADDRESS = 0;	
	
	NRF_RADIO->INTENCLR = 0xFFFF; //clear all
//	NRF_RADIO->INTENSET = 0b01000; //END event
	NRF_RADIO->INTENSET = 0b01010; //END + ADDRESS
	NVIC_SetPriority(RADIO_IRQn, 0);
	NVIC_EnableIRQ(RADIO_IRQn);
	
	ble_conn_mode = ble_mode_advertising;
}

void ble_LL_radio_off()
{
	NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
	NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
	NRF_RADIO->TASKS_DISABLE = 1;
}

volatile uint32_t hop_time_mcs = 0;

uint32_t ble_get_hop_time_mcs()
{
	return hop_time_mcs;
}

void ble_LL_hop()
{
	int had_change = 0;	
	uint8_t chan = ble_LL_get_next_channel(&ll_link);
	if(ll_link_update.interval > 0) //pending change
	{
		if(ll_link_update.instant == ll_link.event_count)
		{
			if(ll_link_update.interval == 0xFFFF) // channel map update
			{
				for(int x = 0; x < 5; x++)
					ll_link.chan_map[x] = ll_link_update.chan_map[x];
				ble_LL_update_channel_map(&ll_link);
				if(!ble_LL_is_channel_used(&ll_link, chan))
					chan = ble_LL_channel_map[chan%ble_LL_channel_count];

				ll_link_update.interval = 0;
				
//				ble_conn_state = ble_conn_init;
//				schedule_event(0x1FFFFF, ble_LL_hop, 1); //effectively pause timer for 2 seconds
//				had_change = 1;
			}
			else
			{
				ll_link.interval = ll_link_update.interval;
				ll_link.timeout = ll_link_update.timeout;
				ll_link.latency = ll_link_update.latency;
				ble_conn_state = ble_conn_init;
				ll_link_update.interval = 0; //clear pending
				schedule_event(0x1FFFFF, ble_LL_hop, 1); //effectively pause timer for 2 seconds
				had_change = 1;
			}
		}
	}
//	schedule_subevent((900+ll_link.win_size*1250)*16, ble_LL_radio_off);
	NRF_RADIO->TASKS_DISABLE = 1;
	NRF_RADIO->BASE0 = ll_link.AA<<8;
	NRF_RADIO->PREFIX0 = ll_link.AA>>24;
	int freq = ble_channel_to_frequency(chan);
	NRF_RADIO->FREQUENCY = freq;
	NRF_RADIO->DATAWHITEIV = (1<<6) | chan; //bit 6 is hardwired to 1 on nRF52832 but it could be not the case on other chips
	NRF_RADIO->CRCPOLY = 0x00065B;
	NRF_RADIO->CRCINIT = ll_link.CRC;
	ll_link.event_count++;

	hop_time_mcs = micros();
	
	NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer;
	if(ll_link.encr.enabled == 3)
	{
		ll_link.encr.enabled = 0;	
		ble_SMP_notify_security(&smp_link, 0);
		ble_encr_ccm_config(0); //off
	}
	if(ll_link.encr.enabled == 2)
	{
		ll_link.encr.enabled = 1;
		ll_link.encr.ccm_data.packet_counter_our = 0;
		ll_link.encr.ccm_data.packet_counter_msb_our = 0;
		ll_link.encr.ccm_data.packet_counter_tgt = 0;
		ll_link.encr.ccm_data.packet_counter_msb_tgt = 0;

		NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer_encr;
		ll_link.encr.ccm_data.direction = 1;
		ble_encr_ccm_fill_array(&ll_link);

		NRF_CCM->INPTR = (uint32_t)ble_rx_buffer_encr;
		NRF_CCM->OUTPTR = (uint32_t)ble_rx_buffer;
		NRF_CCM->SCRATCHPTR = (uint32_t)ble_encr_temp_buf;
		NRF_CCM->MODE = 1 | (1<<24); //decryption, extended length, 1 mbit
		NRF_CCM->SHORTS = 0; //triggered by PPI
		
		ble_encr_ccm_config(1); //rx
		
		ble_SMP_notify_security(&smp_link, 1);
	}
	else if(ll_link.encr.enabled == 1)
	{
		NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer_encr;
		ll_link.encr.ccm_data.direction = 1;
		ble_encr_ccm_fill_array(&ll_link);
		
		NRF_CCM->INPTR = (uint32_t)ble_rx_buffer_encr;
		NRF_CCM->OUTPTR = (uint32_t)ble_rx_buffer;
		NRF_CCM->SCRATCHPTR = (uint32_t)ble_encr_temp_buf;
		NRF_CCM->MODE = 1 | (1<<24); //decryption, extended length, 1 mbit
		ble_encr_ccm_config(1); //rx
	}

	rf_mode_rx_only();
	ble_radio_rxtx = ble_radio_rx;
	
	NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
	
//	uprintf("hop %lu to %d\n", micros(), chan);
#ifdef BLE_DEBUG_PRINTS
	if(had_change) uprintf("hop to %d - new schedule\n", chan);
#endif
	if(ble_conn_state == ble_conn_wait)
	{
		ble_conn_state = ble_conn_init;
		schedule_event(0x2FFFFF, ble_LL_hop, 1); //effectively pause timer for 3 seconds - if nothing will happen, that will be used as timeout
#ifdef BLE_DEBUG_PRINTS
		uprintf("waiting for rx event or timeout (%d)\n", chan, ll_link.timeout);
#endif
	}
	uint32_t ms = millis();
	if(ms - ll_link.last_rx_event_time > 1500 + (ll_link.timeout < 1000?ll_link.timeout*10:10000))
	{
		ble_conn_mode = ble_mode_advertising;
		schedule_event_stop();
		ble_init_radio();
#ifdef BLE_DEBUG_PRINTS
		uprintf("LL timeout: %lu %lu\n", ms, ll_link.last_rx_event_time);
#endif
		if(!ll_link.reached_att_stage)
			ble_ll_same_event_response = !ble_ll_same_event_response;
	}
//	else uprintf("hop to %d ec %d\n", chan, ll_link.event_count);
}

void ble_LL_start_connect()
{
	ll_link.encr.enabled = 0;
	ll_link.last_channel = 0;
	ll_link.SN = 0;
	ll_link.NESN = 0;
	ll_link.our_params_requested = 0;
	ll_link.reached_att_stage = 0;

	smp_link.response_pending = 0;
	smp_link.in_pairing = 0;
	smp_link.expected_key_mask = 0;
	smp_link.is_secure = 0;

	att_link.mtu = ATT_MTU_DEFAULT;
	att_link.response_pending = 0;
	att_link.mtu_change_pending = 0;
	
	
	ble_LL_update_channel_map(&ll_link);
//	ll_link.last_channel = ble_LL_get_next_channel(&ll_link);

	ll_link.event_count = 0;
	ll_link.link_start_time = millis();
	ll_link.last_rx_event_time = millis();
	NRF_RADIO->BASE0 = ll_link.AA<<8;
	NRF_RADIO->PREFIX0 = ll_link.AA>>24;
	int freq = ble_channel_to_frequency(ll_link.last_channel);
	NRF_RADIO->FREQUENCY = freq;
	NRF_RADIO->DATAWHITEIV = (1<<6) | ll_link.last_channel; //bit 6 is hardwired to 1 on nRF52832 but it could be not the case on other chips
	NRF_RADIO->CRCPOLY = 0x00065B;
	NRF_RADIO->CRCINIT = ll_link.CRC;
	

//	schedule_event_delayed((ll_link.offset + ll_link.win_size)*1250*16-3000, ll_link.interval*1250*16, ble_LL_hop, 1); //doesn't work
//	schedule_event_delayed((ll_link.offset + ll_link.win_size)*1250*16, ll_link.interval*1250*16, ble_LL_hop, 1); //sometimes works
	
	if(ll_link.offset > 1)
		schedule_event((ll_link.offset*1250-300)*16, ble_LL_hop, 0); 
	else
		schedule_event((1*1250-300)*16, ble_LL_hop, 0); 
//	schedule_event_delayed((ll_link.offset + ll_link.win_size + ll_link.interval)*1250*16, ll_link.interval*1250*16, ble_LL_hop, 1); //works ok
	
	NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer;
	ble_tx_buffer[1] = 0; //clear up in case if there is some trash
	ble_tx_buffer_pre[0] = 0;
	ble_tx_buffer_pre[1] = 0;

	rf_mode_rx_only();
	NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
	ble_radio_rxtx = ble_radio_rx;
	ble_conn_state = ble_conn_wait;
	ble_conn_mode = ble_mode_connection;
}

ble_LL_data_header1 our_tx_header;
int ble_LL_pending_answer = -1;
uint8_t ble_LL_pending_response[32];

//===============LL packets processors======================
int ble_LL_feature_rsp(volatile uint8_t *tx_buf)
{
	uint8_t feature_set[8] = {0,0,0,0,0,0,0,0};
	feature_set[0] = 0b100001; //LE encryption, extended length
//	feature_set[0] = 0b000001; //LE encryption
	tx_buf[3] = LL_FEATURE_RSP;
	for(int x = 0; x < 8; x++)
		tx_buf[4+x] = feature_set[x];
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLFR send\n");
#endif		
	return 9;
}
int ble_LL_feature_req_active(volatile uint8_t *tx_buf)
{
	uint8_t feature_set[8] = {0,0,0,0,0,0,0,0};
	feature_set[0] = 0b100001; //LE encryption, extended length
//	feature_set[0] = 0b000001; //LE encryption
	tx_buf[3] = LL_FEATURE_REQ;
	for(int x = 0; x < 8; x++)
		tx_buf[4+x] = feature_set[x];
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLFR send\n");
#endif		
	return 9;
}
int ble_LL_version_ind(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_VERSION_IND;
	tx_buf[4] = 0x08; //4.2
//	tx_buf[4] = 0x06; //4.0
	tx_buf[5] = 0xFF; //company identifier, 0x0059 for Nordic
	tx_buf[6] = 0xFF; 
	tx_buf[7] = 0x04;
	tx_buf[8] = 0x00;
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLVID send\n");
#endif		
	return 6;
}
int ble_LL_length_rsp(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_LENGTH_RSP;
	int max_rx_len = 64;
	int max_rx_time = max_rx_len * 8 * 2;
	int max_tx_len = 64;
	int max_tx_time = max_tx_len * 8 * 2;
	tx_buf[4] = max_rx_len;
	tx_buf[5] = max_rx_len>>8;
	tx_buf[6] = max_rx_time;
	tx_buf[7] = max_rx_time>>8;
	tx_buf[8] = max_tx_len;
	tx_buf[9] = max_tx_len>>8;
	tx_buf[10] = max_tx_time;
	tx_buf[11] = max_tx_time>>8;
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLLENGTH send\n");
#endif		
	return 9;
}
int ble_LL_length_req_send(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_LENGTH_REQ;
	int max_rx_len = 115;
	int max_rx_time = max_rx_len * 8 * 2;
	int max_tx_len = 115;
	int max_tx_time = max_tx_len * 8 * 2;
	tx_buf[4] = max_rx_len;
	tx_buf[5] = max_rx_len>>8;
	tx_buf[6] = max_rx_time;
	tx_buf[7] = max_rx_time>>8;
	tx_buf[8] = max_tx_len;
	tx_buf[9] = max_tx_len>>8;
	tx_buf[10] = max_tx_time;
	tx_buf[11] = max_tx_time>>8;
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL LENGTH UPD send\n");
#endif		
	return 9;
}
int ble_LL_enc_rsp(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_ENC_RSP;
	for(int n = 0; n < 8; n++)
		tx_buf[4+n] = ll_link.encr.SKDs[n];
	for(int n = 0; n < 4; n++)
		tx_buf[12+n] = ll_link.encr.IVs[n];
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL_ENC_RSP\n");
#endif
	return 13;
}
int ble_LL_start_enc_req_rsp(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_START_ENC_REQ;

	ble_encr_fill_key(ll_link.encr.LTK);
	uint8_t rand_in[16];
	for(int n = 0; n < 8; n++)
	{
		rand_in[n] = ll_link.encr.SKDm[n];
		rand_in[8+n] = ll_link.encr.SKDs[n];
	}
	ble_encr_e(rand_in, ll_link.encr.ccm_data.key);
	
	ll_link.encr.enabled = 2; //this packet isn't encrypted, but after that turn it on
	ll_link.encr.ccm_data.packet_counter_our = 0;
	ll_link.encr.ccm_data.packet_counter_msb_our = 0;
	ll_link.encr.ccm_data.packet_counter_tgt = 0;
	ll_link.encr.ccm_data.packet_counter_msb_tgt = 0;
	for(int n = 0; n < 4; n++)
	{
		ll_link.encr.ccm_data.IV[n] = ll_link.encr.IVm[n];
		ll_link.encr.ccm_data.IV[4+n] = ll_link.encr.IVs[n];
	}
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL_ENC_START\n");
#endif
	return 1;
}
int ble_LL_start_enc_rsp(volatile uint8_t *tx_buf)
{
	tx_buf[3] = LL_START_ENC_RSP;
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL_START_ENC_RSP sent\n");
#endif
	return 1;
}

int ble_LL_terminate_ind()
{
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL terminate rcvd\n");
#endif
	schedule_event_stop();
	ble_conn_mode = ble_mode_advertising; //fuck it, just drop everything - anyway next connection will re-init all states
	ble_init_radio(); //reset radio parameters to advertisement mode
	return -1;
}
int ble_LL_feature_req()
{
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLFR\n");
#endif
	return LL_FEATURE_RSP;
}
int ble_LL_enc_req()
{
	for(int n = 0; n < 8; n++)
		ll_link.encr.er_rand[n] = ble_rx_buffer[4+n];
	ll_link.encr.EDIV = ble_rx_buffer[12] | (ble_rx_buffer[13]<<8);
	for(int n = 0; n < 8; n++)
		ll_link.encr.SKDm[n] = ble_rx_buffer[14+n];
	for(int n = 0; n < 4; n++)
		ll_link.encr.IVm[n] = ble_rx_buffer[22+n];
	
	ble_rand_fill8(ll_link.encr.SKDs);
	uint32_t rn = ble_rand_num();
	ll_link.encr.IVs[0] = rn;
	ll_link.encr.IVs[1] = rn>>8;
	ll_link.encr.IVs[2] = rn>>16;
	ll_link.encr.IVs[3] = rn>>24;
	
	ll_link.encr.DIV = ll_link.encr.EDIV;
	if(ll_link.encr.DIV != 0)
	{
		ble_encr_restore_keys(&ll_link);
//		ble_encr_generate_keys(&ll_link); //== 0 during pairing, then STK should be used instead
#ifdef BLE_DEBUG_PRINTS	
		uhex_print(ll_link.encr.LTK, 4, 0);
		uprintf(" :LTK\n");
//		for(int n = 0; n < 16; n++)
//			uprintf("%02X", ll_link.encr.LTK[n]);
//		uprintf("\n");
#endif
	}
	
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLENC: %d\n", ll_link.encr.DIV);
#endif
	return LL_ENC_RSP;
}
int ble_LL_start_enc_req()
{
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLSE: %02X\n", ble_rx_buffer[0]);
#endif
	return LL_START_ENC_RSP;
}
int ble_LL_start_enc_rsp_req()
{
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LL_START_ENC_RSP\n");
#endif
	return LL_START_ENC_RSP;
}
int ble_LL_pause_enc_req()
{
	ll_link.encr.enabled = 3; //finalizing
#ifdef BLE_DEBUG_PRINTS		
	uprintf("LLPE: %02X\n", ble_rx_buffer[0]);
#endif
	return LL_PAUSE_ENC_RSP;
}
int ble_LL_version_ind_req()
{
#ifdef BLE_DEBUG_PRINTS
	uprintf("LLVER %02X, %02X%02X\n", ble_rx_buffer[4], ble_rx_buffer[5], ble_rx_buffer[6]);
#endif		
	uint16_t manufacturer_id = (ble_rx_buffer[6]<<8) | ble_rx_buffer[5];
	if(manufacturer_id == 0x0F) //Broadcomm, iphones, macs
	{
		ble_ll_same_event_response = 1;
	}
	if(manufacturer_id == 0x75) //Samsung
	{
		ble_ll_same_event_response = 0;
	}
	return LL_VERSION_IND;
}
int ble_LL_ping_req()
{
#ifdef BLE_DEBUG_PRINTS
	uprintf("LLPNG\n");
#endif
	return LL_PING_RSP;
}
int ble_LL_channel_map_req()
{
	uint8_t pos = 4;
	ll_link_update.chan_map[0] = ble_rx_buffer[pos++];
	ll_link_update.chan_map[1] = ble_rx_buffer[pos++];
	ll_link_update.chan_map[2] = ble_rx_buffer[pos++];
	ll_link_update.chan_map[3] = ble_rx_buffer[pos++];
	ll_link_update.chan_map[4] = ble_rx_buffer[pos++];
	ll_link_update.instant = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	ll_link_update.interval = 0xFFFF;
#ifdef BLE_DEBUG_PRINTS
	uhex_print(ll_link_update.chan_map, 5, 1);
	uprintf(": chan map upd in %d\n", ll_link_update.instant - ll_link.event_count);
#endif
	return -1;
}
int ble_LL_connect_update_req()
{
	uint8_t pos = 4;
	ll_link_update.win_size = ble_rx_buffer[pos];
	pos++;
	ll_link_update.offset = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	pos += 2;
	ll_link_update.interval = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	pos += 2;
	ll_link_update.latency = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	pos += 2;
	ll_link_update.timeout = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	pos += 2;
	ll_link_update.instant = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);

#ifdef BLE_DEBUG_PRINTS
	uprintf("LLCON inst %d ec %d int %d lat %d\n", ll_link_update.instant, ll_link.event_count, ll_link_update.interval, ll_link_update.latency);
#endif
	return -1;
}
int ble_LL_length_req()
{
#ifdef BLE_DEBUG_PRINTS
	uprintf("LL_LEN\n");
#endif
	return LL_LENGTH_RSP;
}
int ble_LL_length_rsp_process()
{
	uint8_t pos = 4;
	int sz1 = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	pos += 2;
	pos += 2;
	int sz2 = ble_rx_buffer[pos] | (ble_rx_buffer[pos+1]<<8);
	ble_update_our_mtu(sz1); //for us it mostly matters how long can be outgoing packets
#ifdef BLE_DEBUG_PRINTS
	uprintf("LL_LEN RSP %d\n", sz1);
#endif
	return -1;
}

//prepare request
int ble_LL_connect_param_req_out(volatile uint8_t *buf, uint16_t int_min, uint16_t int_max, uint16_t latency, uint16_t timeout, uint8_t pref_per, uint16_t cr_count, uint16_t of0, uint16_t of1, uint16_t of2, uint16_t of3, uint16_t of4, uint16_t of5) 
{
	uint8_t pos = 0;
	buf[pos++] = int_min;
	buf[pos++] = int_min>>8;
	buf[pos++] = int_max;
	buf[pos++] = int_max>>8;
	buf[pos++] = latency;
	buf[pos++] = latency>>8;
	buf[pos++] = timeout;
	buf[pos++] = timeout>>8;
	buf[pos++] = pref_per;
	buf[pos++] = cr_count;
	buf[pos++] = cr_count>>8;
	buf[pos++] = of0;
	buf[pos++] = of0>>8;
	buf[pos++] = of1;
	buf[pos++] = of1>>8;
	buf[pos++] = of2;
	buf[pos++] = of2>>8;
	buf[pos++] = of3;
	buf[pos++] = of3>>8;
	buf[pos++] = of4;
	buf[pos++] = of4>>8;
	buf[pos++] = of5;
	buf[pos++] = of5>>8;

#ifdef BLE_DEBUG_PRINTS
	uprintf("LLCON_PARAM req\n");
#endif
	return pos;
}

int ble_LL_unknown_process()
{
//	if(ble_rx_buffer[4] == LL_FEATURE_REQ)
//		return LL_FEATURE_REQ;
	return -1;
}

//===============END OF LL packets processors======================

int ble_LL_process_answer()
{
	if(ble_LL_pending_answer < 0) return 0;
	int resp_length = 0;
	if(ble_LL_pending_answer == LL_FEATURE_REQ)
		resp_length = ble_LL_feature_req_active(ble_tx_buffer);
	if(ble_LL_pending_answer == LL_FEATURE_RSP)
		resp_length = ble_LL_feature_rsp(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_VERSION_IND)
		resp_length = ble_LL_version_ind(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_ENC_RSP)
		resp_length = ble_LL_enc_rsp(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_START_ENC_REQ) //this request is sent from peripheral to central, so for us it is essentially a response
		resp_length = ble_LL_start_enc_req_rsp(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_START_ENC_RSP)
		resp_length = ble_LL_start_enc_rsp(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_LENGTH_RSP)
		resp_length = ble_LL_length_rsp(ble_tx_buffer);
	else if(ble_LL_pending_answer == LL_UNKNOWN_RSP)
	{
		ble_tx_buffer[3] = LL_UNKNOWN_RSP;
		ble_tx_buffer[4] = ble_LL_pending_response[0];
		resp_length = 2;
#ifdef BLE_DEBUG_PRINTS
		uprintf("LL_UNKNOWN_RSP %d sent\n", ble_LL_pending_response[0]);
#endif
	}
	if(resp_length == 0) return 0;
	
	ble_LL_data_header1 tx_hdr;
	tx_hdr.header = 0;
	tx_hdr.LLID = 0b11; //LL command
	tx_hdr.NESN = ll_link.NESN;
	tx_hdr.SN = ll_link.SN;
	tx_hdr.MD = 0;	

	ble_tx_buffer[0] = tx_hdr.header;
	ble_tx_buffer[1] = resp_length;
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;
#ifdef BLE_DEBUG_PRINTS
	uprintf("LL rsp len %d c %d\n", resp_length, ble_tx_buffer[3]);
#endif
		
	ble_radio_rxtx = ble_radio_tx;
	if(ble_LL_pending_answer == LL_ENC_RSP)
		ble_LL_pending_answer = LL_START_ENC_REQ;
	else
		ble_LL_pending_answer = -1;
	return resp_length;
}

void ble_process_LL_control(int is_cc_pack)
{
	ble_LL_data_header1 tx_hdr;
	tx_hdr.header = 0;
	tx_hdr.LLID = 0b01; //by default response is empty PDU
	tx_hdr.NESN = ll_link.NESN;
	tx_hdr.SN = ll_link.SN;
	tx_hdr.MD = 0;

	if(ble_LL_pending_answer >= 0)// && !is_cc_pack)
	{
		ble_LL_process_answer();
		return;
	}
	if(!is_cc_pack) return;
	uint8_t cmd = ble_rx_buffer[3];
	ble_LL_pending_answer = -1;
	if(cmd == LL_TERMINATE_IND) ble_LL_pending_answer = ble_LL_terminate_ind();
	else if(cmd == LL_FEATURE_REQ) ble_LL_pending_answer = ble_LL_feature_req();
	else if(cmd == LL_FEATURE_RSP) ble_LL_pending_answer = -1; //ignore
	else if(cmd == LL_ENC_REQ) ble_LL_pending_answer = ble_LL_enc_req();
	else if(cmd == LL_START_ENC_REQ) ble_LL_pending_answer = ble_LL_start_enc_req();
	else if(cmd == LL_START_ENC_RSP) ble_LL_pending_answer = ble_LL_start_enc_rsp_req(); //this response is sent from central to peripheral, so for us it is a request despite its name
	else if(cmd == LL_PAUSE_ENC_REQ) ble_LL_pending_answer = ble_LL_pause_enc_req();
	else if(cmd == LL_VERSION_IND) ble_LL_pending_answer = ble_LL_version_ind_req();
	else if(cmd == LL_PING_REQ) ble_LL_pending_answer = ble_LL_ping_req();
	else if(cmd == LL_CHANNEL_MAP_REQ) ble_LL_pending_answer = ble_LL_channel_map_req();
	else if(cmd == LL_CONNECTION_UPDATE_REQ) ble_LL_pending_answer = ble_LL_connect_update_req();
	else if(cmd == LL_LENGTH_REQ) ble_LL_pending_answer = ble_LL_length_req();
	else if(cmd == LL_LENGTH_RSP) ble_LL_pending_answer = ble_LL_length_rsp_process();
	else if(cmd == LL_UNKNOWN_RSP) ble_LL_pending_answer = ble_LL_unknown_process();
	else 
	{
		ble_LL_pending_answer = LL_UNKNOWN_RSP;
		ble_LL_pending_response[0] = cmd;
#ifdef BLE_DEBUG_PRINTS
//		uhex_print(ble_rx_buffer, 8, 1);
		uprintf("LL upk\n");
#endif
	}
	if(ble_ll_same_event_response)
		if(ble_LL_process_answer() > 0)
			return;

	ble_tx_buffer[0] = tx_hdr.header;
	ble_tx_buffer[1] = 0;
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;

	ble_radio_rxtx = ble_radio_tx;
}

void ble_process_LL_data()
{
	ble_LL_data_header1 tx_hdr;
	tx_hdr.header = 0;
	tx_hdr.LLID = 0b01;
	tx_hdr.NESN = ll_link.NESN; //rx_hdr.NESN + 1;
//	tx_hdr.SN = rx_hdr.SN + 1;
	tx_hdr.SN = ll_link.SN; //rx_hdr.SN;
	tx_hdr.MD = 0;

	ble_tx_buffer[0] = tx_hdr.header;
	ble_tx_buffer[1] = 0;
	ble_tx_buffer[2] = 0; //S1 - unused
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;
	ble_radio_rxtx = ble_radio_tx;
	if(ble_rx_buffer[1] > 3) //min l2cap packet 4 bytes
	{
		sL2CAP_header l2_hdr;
		l2_hdr.length = ble_rx_buffer[3] | (ble_rx_buffer[4]<<8);
		l2_hdr.CID = ble_rx_buffer[5] | (ble_rx_buffer[6]<<8);
		if(l2_hdr.CID == 4)
		{
			ble_process_ATT(l2_hdr.length, ble_rx_buffer+7, &att_link);
			ll_link.reached_att_stage = 1;
		}
		else if(l2_hdr.CID == 6)
		{
			ble_process_SMP(l2_hdr.length, ble_rx_buffer+7, &smp_link, &ll_link);
		}
		else
		{
#ifdef BLE_DEBUG_PRINTS
			uprintf("unhandled CID %d ", l2_hdr.CID); 
			uhex_print(ble_rx_buffer+3, ble_rx_buffer[1], 1);
//			for(int n = 0; n < ble_rx_buffer[1]; n++)
//				uprintf("%02X", ble_rx_buffer[3+n]);
			uprintf("\n");
#endif
			
		}
	}
	if(ble_rx_buffer[1] == 0) //empty pdu
	{
		if(att_link.response_pending)
		{
			tx_hdr.LLID = 0b10;
			ble_tx_buffer[0] = tx_hdr.header;
			
			sL2CAP_header l2_resp_hdr;
			l2_resp_hdr.CID = 4;
			l2_resp_hdr.length = att_link.response_length;
			ble_tx_buffer[1] = l2_resp_hdr.length + 4;
			int pos = 3; //S1 field skipped
			ble_tx_buffer[pos++] = l2_resp_hdr.length;
			ble_tx_buffer[pos++] = l2_resp_hdr.length>>8;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID>>8;
			for(int x = 0; x < att_link.response_length; x++)
				ble_tx_buffer[pos++] = att_link.response_buf[x];
			att_link.response_pending = 0;
#ifdef BLE_DEBUG_PRINTS
			uhex_print(att_link.response_buf, att_link.response_length, 1);
			uprintf(": att resp\n");//%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", ble_tx_buffer[0], ble_tx_buffer[1], ble_tx_buffer[2], ble_tx_buffer[3], ble_tx_buffer[4], ble_tx_buffer[5], ble_tx_buffer[6], ble_tx_buffer[7], ble_tx_buffer[8], ble_tx_buffer[9], ble_tx_buffer[10], ble_tx_buffer[11], ble_tx_buffer[12], ble_tx_buffer[13]);
#endif
		}
		else if(smp_link.response_pending)
		{
			tx_hdr.LLID = 0b10;
			ble_tx_buffer[0] = tx_hdr.header;
			
			sL2CAP_header l2_resp_hdr;
			l2_resp_hdr.CID = 6;
			l2_resp_hdr.length = smp_link.response_length;
			ble_tx_buffer[1] = l2_resp_hdr.length + 4;
			int pos = 3; //S1 field skipped
			ble_tx_buffer[pos++] = l2_resp_hdr.length;
			ble_tx_buffer[pos++] = l2_resp_hdr.length>>8;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID>>8;
			for(int x = 0; x < smp_link.response_length; x++)
				ble_tx_buffer[pos++] = smp_link.response_buf[x];
			smp_link.response_pending = 0;			
#ifdef BLE_DEBUG_PRINTS
			uhex_print(smp_link.response_buf, smp_link.response_length, 1);
			uprintf(": smp resp\n");
#endif
		}
		else if(ble_check_ATT(&att_link))
		{
			tx_hdr.LLID = 0b10;
			ble_tx_buffer[0] = tx_hdr.header;
			
			sL2CAP_header l2_resp_hdr;
			l2_resp_hdr.CID = 4;
			l2_resp_hdr.length = att_link.response_length;
			ble_tx_buffer[1] = l2_resp_hdr.length + 4;
			int pos = 3; //S1 field skipped
			ble_tx_buffer[pos++] = l2_resp_hdr.length;
			ble_tx_buffer[pos++] = l2_resp_hdr.length>>8;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID>>8;
			for(int x = 0; x < att_link.response_length; x++)
				ble_tx_buffer[pos++] = att_link.response_buf[x];
			att_link.response_pending = 0;
#ifdef BLE_DEBUG_PRINTS
			uprintf("att ntf\n");// %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", ble_tx_buffer[0], ble_tx_buffer[1], ble_tx_buffer[2], ble_tx_buffer[3], ble_tx_buffer[4], ble_tx_buffer[5], ble_tx_buffer[6], ble_tx_buffer[7], ble_tx_buffer[8], ble_tx_buffer[9], ble_tx_buffer[10], ble_tx_buffer[11], ble_tx_buffer[12], ble_tx_buffer[13]);
#endif
		}
		else if(ble_check_SMP(&smp_link, &ll_link))
		{
			tx_hdr.LLID = 0b10;
			ble_tx_buffer[0] = tx_hdr.header;
			
			sL2CAP_header l2_resp_hdr;
			l2_resp_hdr.CID = 6;
			l2_resp_hdr.length = smp_link.response_length;
			ble_tx_buffer[1] = l2_resp_hdr.length + 4;
			int pos = 3; //S1 field skipped
			ble_tx_buffer[pos++] = l2_resp_hdr.length;
			ble_tx_buffer[pos++] = l2_resp_hdr.length>>8;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID;
			ble_tx_buffer[pos++] = l2_resp_hdr.CID>>8;
			for(int x = 0; x < smp_link.response_length; x++)
				ble_tx_buffer[pos++] = smp_link.response_buf[x];
			smp_link.response_pending = 0;
#ifdef BLE_DEBUG_PRINTS
			uprintf("smp proc\n");
//			uprintf("smp sent %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", ble_tx_buffer[0], ble_tx_buffer[1], ble_tx_buffer[2], ble_tx_buffer[3], ble_tx_buffer[4], ble_tx_buffer[5], ble_tx_buffer[6], ble_tx_buffer[7], ble_tx_buffer[8], ble_tx_buffer[9], ble_tx_buffer[10], ble_tx_buffer[11], ble_tx_buffer[12], ble_tx_buffer[13]);
#endif
		}
	}
	
#ifdef BLE_DEBUG_PRINTS
	if(ble_rx_buffer[1] > 3) 
	{
//		uhex_print(ble_rx_buffer+3, ble_rx_buffer[1], 1);
//		uprintf(":LLdata\n");
	}
//		uprintf("LLdata %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", ble_rx_buffer[0], ble_rx_buffer[1], ble_rx_buffer[2], ble_rx_buffer[3], ble_rx_buffer[4], ble_rx_buffer[5], ble_rx_buffer[6], ble_rx_buffer[7], ble_rx_buffer[8], ble_rx_buffer[9], ble_rx_buffer[10], ble_rx_buffer[11], ble_rx_buffer[12], ble_rx_buffer[13]);
#endif
	return;
}

void ble_LL_empty_PDU()
{
	ble_LL_data_header1 tx_hdr;
	tx_hdr.header = 0;
	tx_hdr.LLID = 0b01;
	tx_hdr.NESN = ll_link.NESN; //rx_hdr.NESN + is_ok;
	tx_hdr.SN = ll_link.SN;// rx_hdr.SN;
	tx_hdr.MD = 0;

	ble_tx_buffer[0] = tx_hdr.header;
	ble_tx_buffer[1] = 0;
	ble_tx_buffer[2] = 0; //S1 field
	NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer;
	ble_radio_rxtx = ble_radio_tx;
}

int encr_cons_err = 0;

volatile uint32_t avg_hop_to_rx = 800<<8;
volatile uint32_t rx_micros = 0;
int hop_noadjust_cnt = 0;

void ble_conn_irq(int is_address)
{
	if(is_address)
	{
		if(ble_radio_rxtx == ble_radio_tx)
		{
			NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
//			NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
		}
		return;
	}
	if(ble_radio_rxtx == ble_radio_rx)
	{		
		rx_micros = micros();
		int dt = rx_micros - hop_time_mcs;
		if(dt > 650 && dt < 950) //only normal events
		{ //((32000)×(FF«8)+(330«8)×(102))»10
			uint32_t udt = dt<<8;
			avg_hop_to_rx = (avg_hop_to_rx*253 + udt*3)>>8;
		}
		hop_noadjust_cnt++;
		int aht = avg_hop_to_rx>>8;
		if(aht > 850 && hop_noadjust_cnt > 5 && ll_link.event_count > 100)
		{
			schedule_event_adjust(10*16);
			hop_noadjust_cnt = 0;
//			uprintf("adj+ : ");
		}
		if(aht < 750 && hop_noadjust_cnt > 5 && ll_link.event_count > 100)
		{
			schedule_event_adjust(-10*16);
			hop_noadjust_cnt = 0;
//			uprintf("adj- : ");
		}
//		if(ble_rx_buffer[1] > 0)
//			uprintf("edt %d adt %d\n", dt, aht);
		

		if(ble_conn_state == ble_conn_init)
		{
			schedule_event_delayed((ll_link.interval*1250-800)*16, ll_link.interval*1250*16, ble_LL_hop, 1); 
			ble_conn_state = ble_conn_ok;
		}
		ll_link.last_rx_event_time = millis();

		uint8_t encr_err = 0;
		if(ll_link.encr.enabled && ble_rx_buffer[1] > 0)
		{
			if(NRF_CCM->MICSTATUS == 0)
			{
				encr_err = 1;
				encr_cons_err++;
				ble_LL_empty_PDU();
#ifdef BLE_DEBUG_PRINTS
				uprintf("ll encr mic err %d\n", encr_cons_err); //%d (%02X %02X %02X %02X / %02X %02X %02X %02X)\n", NRF_CCM->MICSTATUS, ble_rx_buffer[0], ble_rx_buffer[1], ble_rx_buffer[3], ble_rx_buffer[4], ble_rx_buffer_encr[0], ble_rx_buffer_encr[1], ble_rx_buffer_encr[3], ble_rx_buffer_encr[4]);	
#endif
				goto tx_encryption_part;
			}
			encr_cons_err = 0;
			ll_link.encr.ccm_data.packet_counter_tgt++;
			if(ll_link.encr.ccm_data.packet_counter_tgt == 0) ll_link.encr.ccm_data.packet_counter_msb_tgt++;
		}
		int plen = ble_rx_buffer[1];
//		for(int x = 0; x < plen+2; x++)
//			ble_last_rx_pack[x] = ble_rx_buffer[x];
//		ble_last_rx_pack_len = plen;
//		ble_last_rx_pack_id++;
//		ble_last_rx_pack_crc = NRF_RADIO->CRCSTATUS;


		ble_tx_buffer[1] = 0; //just in case, if we will miss something with other processing

		if(!NRF_RADIO->CRCSTATUS)
		{
			ble_LL_empty_PDU();
#ifdef BLE_DEBUG_PRINTS
			uprintf("CRC ERR\n");
#endif			
			goto tx_encryption_part;
		}

		ble_LL_data_header1 rx_hdr;
		rx_hdr.header = ble_rx_buffer[0];
		int old_packet = 0;
		int need_resend = 0;
		if(rx_hdr.SN != ll_link.NESN)
		{
			old_packet = 1;
#ifdef BLE_DEBUG_PRINTS
			uprintf("old pack\n");
#endif			
		}
		if(rx_hdr.NESN == ll_link.SN)
		{
			need_resend = 1;
#ifdef BLE_DEBUG_PRINTS
			uprintf("need resend\n");
#endif			
		}
		if(old_packet) //need to ignore it completely
		{
			NRF_RADIO->SHORTS &= ~(RADIO_SHORTS_DISABLED_TXEN_Msk);
			return;
		}
		NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk; //if not an old packet, need to send something

		if(need_resend)
		{
			for(int x = 0; x < ble_tx_buffer_pre[1]+3; x++)
				ble_tx_buffer[x] = ble_tx_buffer_pre[x];
			goto tx_encryption_part;
		}
		
		ll_link.NESN = !ll_link.NESN;
		ll_link.SN = rx_hdr.SN;
		

		if((rx_hdr.LLID == 0b11 && plen > 0) || (ble_LL_pending_answer >= 0 && plen == 0)) ble_process_LL_control(rx_hdr.LLID == 0b11);
		else if(rx_hdr.LLID == 0b01 || rx_hdr.LLID == 0b10) ble_process_LL_data();
		else if(rx_hdr.LLID == 0)
		{
#ifdef BLE_DEBUG_PRINTS
			uhex_print(ble_rx_buffer, ble_rx_buffer[1] + 3, 1);
			uprintf(":zero LLID\n");//, ble_rx_buffer[0], ble_rx_buffer[1], ble_rx_buffer[2], ble_rx_buffer[3]);
#endif
			ble_LL_empty_PDU();
		}
/*		
		static int feature_req_sent = 0;
		if(0)if(!feature_req_sent && ble_tx_buffer[1] == 0)
		{
			feature_req_sent = 1;
			ble_LL_feature_req_active(ble_tx_buffer);
		}
		static int empty_exch_cnt = 0;
		if(ble_rx_buffer[1] == 0 && ble_tx_buffer[1] == 0) //empty PDUs exchange - can use for our requests
			empty_exch_cnt++;
		else empty_exch_cnt = 0;
		
		if(0)if(empty_exch_cnt > 2)
		{
			if(ll_link.our_params_requested == 0) 
			{
				ble_LL_data_header1 tx_hdr;
				tx_hdr.header = 0;
				tx_hdr.LLID = 0b11;
				tx_hdr.NESN = ll_link.NESN; //rx_hdr.NESN + is_ok;
				tx_hdr.SN = ll_link.SN;// rx_hdr.SN;
				tx_hdr.MD = 0;

				ble_tx_buffer[0] = tx_hdr.header;
				ble_tx_buffer[1] = ble_LL_connect_param_req_out(ble_tx_buffer+3, 6, 24, 0, 210, 6, ll_link.event_count, 16, 25, 24, 32, 65, 128);
//				ble_tx_buffer[1] = ble_LL_length_req_send(ble_tx_buffer);
				ll_link.our_params_requested = 1;
			}
		}
*/
tx_encryption_part:
//====ENCRYPTION IS CONVENIENT TO APPLY HERE, WHEN EVERYTHING IS PROCESSED AND ble_tx_buffer IS FILLED===========
		if(!old_packet) //store as last sent only if it's a valid RX packet
			for(int x = 0; x < ble_tx_buffer[1]+3; x++) //storing old packet in case if need to retransmit
				ble_tx_buffer_pre[x] = ble_tx_buffer[x];

		if(0)if(ll_link.encr.enabled == 1 || ll_link.encr.enabled == 3)
		{			
			NRF_RADIO->PACKETPTR = (uint32_t)ble_tx_buffer_encr;
			ll_link.encr.ccm_data.direction = 0;
			ble_encr_ccm_fill_array(&ll_link);
			
			NRF_CCM->INPTR = (uint32_t)ble_tx_buffer;
			NRF_CCM->OUTPTR = (uint32_t)ble_tx_buffer_encr;
			NRF_CCM->SCRATCHPTR = (uint32_t)ble_encr_temp_buf;
			NRF_CCM->MODE = 0 | (1<<24); //encryption, extended length, 1 mbit
			ble_encr_ccm_config(2); //tx

			if(ble_tx_buffer[1] > 0)
			{
				ll_link.encr.ccm_data.packet_counter_our++;
				if(ll_link.encr.ccm_data.packet_counter_our == 0) ll_link.encr.ccm_data.packet_counter_msb_our++;
			}
		}
		ble_radio_rxtx = ble_radio_tx;

#ifdef BLE_DEBUG_PRINTS
		if(0)if(ble_rx_buffer[1] >= 1)
		{
			uhex_print(ble_rx_buffer, ble_rx_buffer[1] + 3, 1);
			uprintf(":rx %d\n", micros() - rx_micros);
		}
#endif
		return;
	}
	if(ble_radio_rxtx == ble_radio_tx)
	{
#ifdef BLE_DEBUG_PRINTS		
		if(0)if(ble_tx_buffer[1] > 0)
		{
			uhex_print(ble_tx_buffer, ble_tx_buffer[1]+3, 1);
			uprintf(": tx %d\n", micros() - rx_micros);
		}
#endif
		NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer;
		if(0)if(ll_link.encr.enabled == 1)
		{
			NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer_encr;
			ll_link.encr.ccm_data.direction = 1;
			ble_encr_ccm_fill_array(&ll_link);
			
			NRF_CCM->INPTR = (uint32_t)ble_rx_buffer_encr;
			NRF_CCM->OUTPTR = (uint32_t)ble_rx_buffer;
			NRF_CCM->SCRATCHPTR = (uint32_t)ble_encr_temp_buf;
			NRF_CCM->MODE = 1 | (1<<24); //decryption, extended length, 1 mbit
			ble_encr_ccm_config(1); //rx
		}		
		return;
	}
}

void ble_adv_irq()
{
	if(ble_radio_rxtx & ble_radio_tx) //sending complete
	{
		NRF_RADIO->PACKETPTR = (uint32_t)ble_rx_buffer;
		if(ble_is_connectable) rf_mode_rx_only();
		NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
		NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
		ble_radio_rxtx = ble_radio_rx;
		ble_conn_state = ble_adv_sent;
		return;
	}
	if(ble_radio_rxtx & ble_radio_rx)
	{
		ble_adv_header1 hdr;
		hdr.header = ble_rx_buffer[0];
		int plen = ble_rx_buffer[1];
		for(int x = 0; x < plen+2; x++)
			ble_last_rx_pack[x] = ble_rx_buffer[x];
		ble_last_rx_pack_len = plen;
		ble_last_rx_pack_id++;
		ble_last_rx_pack_crc = NRF_RADIO->CRCSTATUS;
		
		if(hdr.type == BLE_SCAN_REQ_TYPE && ble_conn_state == ble_adv_sent)
		{
			int prev_ble_state = ble_conn_state;
			ble_conn_state = ble_adv_scan_request;
			int our_dev = 1;
			for(int x = 0; x < 6; x++)
				if(ll_link.our_mac[x] != ble_rx_buffer[3+6+x]) our_dev = 0;
			if(!our_dev || !ble_is_connectable)
			{
				ble_conn_state = prev_ble_state;
				if(ble_is_connectable) rf_mode_rx_only(); //continue listening
#ifdef BLE_DEBUG_PRINTS
				uprintf("scan req to another device\n");
#endif
				return;
			}
//			uprintf("scan req - responding\n");
			for(int x = 0; x < 6; x++)
				incoming_mac[x] = ble_rx_buffer[3+x];
			uint8_t pdu[40];
			int pdu_len = ble_prepare_adv_pdu(pdu, 6, ll_link.our_mac, BLE_SCAN_RSP_TYPE, hdr.tx_addr, 1);
			ble_LL_respond_PDU(BLE_ADV_ADDR, pdu_len, pdu, ble_current_channel);
			ble_conn_state = ble_adv_scan_response;
			return;
		}
		if(hdr.type == BLE_CONNECT_REQ_TYPE)// && ble_conn_state == ble_adv_scan_response)
		{
			int our_dev = 1;
			for(int x = 0; x < 6; x++)
				if(ll_link.our_mac[x] != ble_rx_buffer[3+6+x]) our_dev = 0;
			if(!our_dev)
			{
				ble_conn_state = ble_adv_sent;
				rf_mode_rx_only(); //continue listening
				return;
			}
			for(int x = 0; x < 6; x++)
			{
				incoming_mac[x] = ble_rx_buffer[3+x];
				ll_link.tgt_mac[x] = ble_rx_buffer[3+x];
			}
			ll_link.our_addr_type = hdr.rx_addr;
			ll_link.tgt_addr_type = hdr.tx_addr;
			
			ll_link.AA = 0;
			int lppos = 3+6+6;
			for(int x = 0; x < 4; x++)
				ll_link.AA |= ((uint32_t)ble_rx_buffer[lppos+x])<<(x*8);
			lppos += 4;
			ll_link.CRC = 0;
			for(int x = 0; x < 3; x++)
				ll_link.CRC |= ((uint32_t)ble_rx_buffer[lppos+x])<<(x*8);
			lppos += 3;
			ll_link.win_size = ble_rx_buffer[lppos];
			lppos++;
			ll_link.offset = ble_rx_buffer[lppos] | (ble_rx_buffer[lppos+1]<<8);
			lppos += 2;
			ll_link.interval = ble_rx_buffer[lppos] | (ble_rx_buffer[lppos+1]<<8);
			lppos += 2;
			ll_link.latency = ble_rx_buffer[lppos] | (ble_rx_buffer[lppos+1]<<8);
			lppos += 2;
			ll_link.timeout = ble_rx_buffer[lppos] | (ble_rx_buffer[lppos+1]<<8);
			lppos += 2;
			for(int x = 0; x < 5; x++)
				ll_link.chan_map[x] = ble_rx_buffer[lppos++];
//			ll_link.hop = ble_rx_buffer[lppos]>>3;
//			ll_link.sca = ble_rx_buffer[lppos]&0b111;
			ll_link.hop = ble_rx_buffer[lppos]&0b11111;
			ll_link.sca = ble_rx_buffer[lppos]>>5;
			ll_link_update.interval = 0; //mark as unused
			if(ll_link.offset > 10000) //suspiciously long, igonre it
				return;
			ble_LL_start_connect();
			ble_clear_notifications(&att_link);
#ifdef BLE_DEBUG_PRINTS
			uprintf("ll connect request hop %d sca %d lat %d int %d ws %d offs %d to %d ch %02X %02X %02X %02X %02X\n", ll_link.hop, ll_link.sca, ll_link.latency, ll_link.interval, ll_link.win_size, ll_link.offset, ll_link.timeout, ll_link.chan_map[0], ll_link.chan_map[1], ll_link.chan_map[2], ll_link.chan_map[3], ll_link.chan_map[4]);
#endif
			return;
		}
		if(ble_is_connectable) rf_mode_rx_only(); //default: continue listening
	}
}

void *ble_get_ll_link()
{
	return &ll_link;
}
int ble_get_conn_state()
{
	return ble_conn_mode == ble_mode_connection;
}

void ble_radio_irq()
{
	uint8_t handled = 0;
	if(NRF_RADIO->EVENTS_ADDRESS)
	{
		handled = 1;
		NRF_RADIO->EVENTS_ADDRESS = 0;
		if(ble_conn_mode == ble_mode_connection)
			ble_conn_irq(1);
	}
	if(NRF_RADIO->EVENTS_END)// && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk))
	{
		handled = 1;
		NRF_RADIO->EVENTS_END = 0;
		if(ble_conn_mode == ble_mode_advertising)
		{
			ble_adv_irq();
			return;
		}
		if(ble_conn_mode == ble_mode_connection)
		{
			ble_conn_irq(0);
			return;
		}
	}
	if(!handled)
	{
		rf_clear_events();
	}
}

uint32_t ble_get_last_rx_id()
{
	return ble_last_rx_pack_id;
}
int ble_get_last_rx_pack(uint8_t *buf, int max_buf_len)
{
	int rlen = ble_last_rx_pack_len;
	if(rlen > max_buf_len) rlen = max_buf_len;
	for(int x = 0; x < rlen; x++)
		buf[x] = ble_last_rx_pack[x];
	return rlen;
}
int ble_get_last_rx_pack_crc()
{
	return ble_last_rx_pack_crc;
}

void ble_peripheral_set_ER(uint8_t *er_text)
{
	ble_encr_set_ER(er_text, &ll_link);
}
void ble_peripheral_set_IR(uint8_t *ir_text)
{
	ble_encr_set_IR(ir_text, &ll_link);
}
void ble_peripheral_generate_keys()
{
	ble_encr_generate_keys(&ll_link);
}

void ble_peripheral_generate_mac(uint8_t *res)
{
	ble_peripheral_generate_keys();
	uint32_t rv = NRF_FICR->DEVICEID[1];//ble_rand_num();
	//5C1AC9270389
	rv &= 0x3FFFFF;
	rv |= 1<<22;
	uint32_t rv2 = ble_encr_ah(ll_link.encr.IRK, rv);
	res[0] = rv2;
	res[1] = rv2>>8;
	res[2] = rv2>>16;
	res[3] = rv;
	res[4] = rv>>8;
	res[5] = rv>>16;	
}

int ble_peripheral_in_pairing()
{
	return smp_link.in_pairing;
}

int ble_get_current_MTU()
{
	return att_link.mtu;
}
