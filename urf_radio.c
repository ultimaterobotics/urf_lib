#include "urf_radio.h"
#include "urf_timer.h"
#include "nrf.h"
#include <stdlib.h> //NULL

uint32_t pack_max_length = 255;  //maximum length in bytes
uint32_t pack_payload;  //current payload size in bytes

uint32_t rx_packet_counter = 0;
uint32_t last_processed_rx_packet = 0;

volatile uint8_t current_mode = 0;
volatile int ack_state = 0;

uint8_t rx_packet[256];
uint8_t tx_packet[256];

uint8_t resp_crc_ok[32];
uint8_t resp_crc_fail[32];

uint8_t resp_length = 0;
uint8_t auto_respond = 0;

void (*rf_irq_override)() = NULL;
void (*rf_rx_irq)() = NULL;
void (*rf_tx_irq)() = NULL;

volatile int rf_busy = 0;

void rf_disable()
{
	if(NRF_RADIO->STATE == st_radio_disabled) return; //already disabled
	if(NRF_RADIO->STATE == st_radio_rxdisable || NRF_RADIO->STATE == st_radio_txdisable)
	{
		while(NRF_RADIO->STATE == st_radio_rxdisable) ;
		while(NRF_RADIO->STATE == st_radio_txdisable) ;
		return;
	}
	
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE   = 1;
	while(NRF_RADIO->STATE != st_radio_disabled) ;
    NRF_RADIO->EVENTS_DISABLED = 0;
}

void rf_init_ext(int channel, int speed, int crc_len, int white_en, int s1_sz, int added_length, int max_length)
{
	NVIC_DisableIRQ(RADIO_IRQn);
	NRF_RADIO->POWER = 1;
    // Radio config
    NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->FREQUENCY = channel;
	if(speed == 250)
		NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);
	if(speed == 1000)
		NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
	if(speed == 1001)
		NRF_RADIO->MODE      = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);
	if(speed == 2000)
		NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);

    // Radio address config
    NRF_RADIO->PREFIX0     = 0x44332211UL;  // Prefix byte of addresses 3 to 0
    NRF_RADIO->PREFIX1     = 0x88776655UL;  // Prefix byte of addresses 7 to 4
    NRF_RADIO->BASE0       = 0x0EE60DA7UL;  // Base address for prefix 0
    NRF_RADIO->BASE1       = 0x0EE60DA7UL;  // Base address for prefix 1-7
    NRF_RADIO->TXADDRESS   = 0x0UL;        // Set device address 0 to use when transmitting
    NRF_RADIO->RXADDRESSES = 0b00000001;        // receive on address 0

	NRF52_PCNF0_REG conf0;
	conf0.reg = 0;
	conf0.length_bitsz = 8;
	conf0.S0_bytesz = 1;
	conf0.S1_bitsz = s1_sz;
	conf0.S1_include = 0;
	conf0.preamble_length = 0;
	
    // Packet configuration
    NRF_RADIO->PCNF0 = conf0.reg;
	
	NRF52_PCNF1_REG conf1;
	conf1.reg = 0;
	conf1.max_length = max_length;
	conf1.added_length = added_length; 
	conf1.addr_len = 4;
	conf1.endian = 1; //big endian is traditional default
	conf1.whitening = white_en;
	
    // Packet configuration
    NRF_RADIO->PCNF1 = conf1.reg;

	NRF_RADIO->TIFS = 30;

    // CRC Config
	NRF_RADIO->CRCCNF = crc_len;
	NRF_RADIO->CRCINIT = 0b10101010;   // Initial value
	NRF_RADIO->CRCPOLY = 0b100000111;  // CRC poly: x^8+x^2+x^1+1
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_END = 0; 

	NRF_RADIO->INTENCLR = 0xFFFFFFFF;
	NRF_RADIO->INTENSET = 0b01000; //END event
//	NRF_RADIO->INTENSET = 0b01010; //END & ADDRESS events
	rf_irq_override = NULL;
	NVIC_SetPriority(RADIO_IRQn, 0);
	NVIC_EnableIRQ(RADIO_IRQn);
}

void rf_init(int channel, int speed, int crc_len)
{
	rf_init_ext(channel, speed, crc_len, 1, 0, 0, pack_max_length);
}

void rf_mode_rx_only()
{
//	while(rf_busy) ;
	if(!(NRF_RADIO->STATE == st_radio_rx || NRF_RADIO->STATE == st_radio_rxru || NRF_RADIO->STATE == st_radio_rxidle))
	{
		rf_disable();
	}
	
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	NRF_RADIO->TASKS_RXEN = 1;
	current_mode = rm_rx2rx;
}
void rf_mode_tx_only()
{
//	while(rf_busy) ;
	if(!(NRF_RADIO->STATE == st_radio_txru || NRF_RADIO->STATE == st_radio_txidle))
	{
		rf_disable();
	}
	
	ack_state = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->TASKS_TXEN = 1;
	current_mode = rm_tx_end;
	rf_busy = 1;
}
void rf_mode_tx_then_rx()
{
	uint32_t start_ms;
	if(rf_busy) start_ms = millis();
	while(rf_busy && millis() - start_ms < 3) ;

	if(!(NRF_RADIO->STATE == st_radio_txru || NRF_RADIO->STATE == st_radio_txidle))
	{
		rf_disable();
	}

	ack_state = 0;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NRF_RADIO->TASKS_TXEN = 1;
	current_mode = rm_tx2rx;
	while(NRF_RADIO->STATE == st_radio_disabled || NRF_RADIO->STATE == st_radio_txdisable || NRF_RADIO->STATE == st_radio_rxdisable) ;
	NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
	rf_busy = 1;
}

void rf_send(uint8_t *pack, int length)
{
	for(int x = 0; x < length; x++)
		tx_packet[x] = pack[x];

	uint32_t start_ms;
	if(rf_busy) start_ms = millis();
	while(rf_busy && millis() - start_ms < 3) ;

	NRF_RADIO->PACKETPTR = (uint32_t)tx_packet;
	rf_mode_tx_only();
}
void rf_listen()
{
	last_processed_rx_packet = rx_packet_counter;
//	while(rf_busy) ;
	NRF_RADIO->PACKETPTR = (uint32_t)rx_packet;
	rf_mode_rx_only();
}
void rf_send_and_listen(uint8_t *pack, int length)
{
	uint32_t *pp = (uint32_t*)pack;
	uint32_t *tp = (uint32_t*)tx_packet;
	for(int x = 0; x < (length>>2)+1; x++)
		*tp++ = *pp++;
	
//	for(int x = 0; x < length; x++)
//		tx_packet[x] = pack[x];
	uint32_t start_ms;
	if(rf_busy) start_ms = millis();
	while(rf_busy && millis() - start_ms < 3) ;
	NRF_RADIO->PACKETPTR = (uint32_t)tx_packet;
	rf_mode_tx_then_rx();
}
int rf_has_new_packet()
{
	return (last_processed_rx_packet != rx_packet_counter);
}
uint32_t rf_get_packet(uint8_t *pack)
{
//	for(int x = 0; x < rx_packet[1]; x++)
//		pack[x] = rx_packet[x];
	uint32_t *pp = (uint32_t*)pack;
	uint32_t *rp = (uint32_t*)rx_packet;
	for(int x = 0; x < (rx_packet[1]>>2)+1; x++)
		*pp++ = *rp++;
	last_processed_rx_packet = rx_packet_counter;
	return rx_packet[1];
}

void rf_autorespond_on(uint8_t *pack_crc_ok, uint8_t *pack_crc_fail, int resp_pack_length)
{
	resp_length = resp_pack_length;
	if(resp_length > 30) resp_length = 30;
	resp_crc_ok[0] = 111;
	resp_crc_ok[1] = resp_length+2;
	resp_crc_fail[0] = 100;
	resp_crc_fail[1] = resp_length+2;
	for(int x = 0; x < resp_length; x++)
	{
		resp_crc_ok[2+x] = pack_crc_ok[x];
		resp_crc_fail[2+x] = pack_crc_fail[x];
	}
	
	auto_respond = 1;
}
void rf_autorespond_off()
{
	auto_respond = 0;
}

void rf_override_irq(void (*new_irq))
{
	rf_irq_override = new_irq;
}

void rf_attach_rx_irq(void (*rx_irq))
{
	rf_rx_irq = rx_irq;
}
void rf_dettach_rx_irq()
{
	rf_rx_irq = NULL;
}

void rf_attach_tx_irq(void (*tx_irq))
{
	rf_tx_irq = tx_irq;
}
void rf_dettach_tx_irq()
{
	rf_tx_irq = NULL;
}



void rf_clear_events()
{
	NRF_RADIO->EVENTS_READY = 0;
	NRF_RADIO->EVENTS_ADDRESS = 0;
	NRF_RADIO->EVENTS_PAYLOAD = 0;
	NRF_RADIO->EVENTS_END = 0;
	NRF_RADIO->EVENTS_DISABLED = 0;
	NRF_RADIO->EVENTS_DEVMATCH = 0;
	NRF_RADIO->EVENTS_DEVMISS = 0;
	NRF_RADIO->EVENTS_RSSIEND = 0;
	NRF_RADIO->EVENTS_BCMATCH = 0;
	NRF_RADIO->EVENTS_CRCOK = 0;
	NRF_RADIO->EVENTS_CRCERROR = 0;	
	rf_busy = 0;
}

void RADIO_IRQHandler()
{
	if(rf_irq_override != NULL)
	{
		rf_irq_override();
		return;
	}
	uint8_t handled = 0;
	uint8_t need_call_tx_cplt = 0;
	uint8_t need_call_rx_cplt = 0;
	if(NRF_RADIO->EVENTS_ADDRESS)
	{
		handled = 1;
//		rf_busy = 1; temporary turn off busy indication
		NRF_RADIO->EVENTS_ADDRESS = 0;
	}
	if(NRF_RADIO->EVENTS_END)// && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk))
	{
		handled = 1;
		rf_busy = 0;
		NRF_RADIO->EVENTS_END = 0;
		if(current_mode == rm_tx_end)
		{
			need_call_tx_cplt = 1;
		}
		else if(current_mode == rm_rx_end)
		{
			rx_packet_counter++;
			need_call_rx_cplt = 1;
		}
		else if(current_mode == rm_tx2rx)
		{
//			uprintf("switching to rx, %d\n", rx_packet_counter);
//			rx_packet_counter++;
			//we are somwhere here    vvvvvvvvvvvvvvv
			//                TX END -> DISABLE -> RXEN
			NRF_RADIO->PACKETPTR = (uint32_t)rx_packet;
			while(NRF_RADIO->STATE != st_radio_rxidle && NRF_RADIO->STATE != st_radio_rx) ;//  && NRF_RADIO->STATE != st_radio_rxru) ;
			NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk; //no need to END->DISABLE with DISABLED->RXEN
//			rf_listen();
			current_mode = rm_rx2rx;
			need_call_tx_cplt = 1;
		}
		else if(current_mode == rm_rx2rx)
		{
			rx_packet_counter++;
			if(auto_respond)
			{
				if(NRF_RADIO->CRCSTATUS != 0)
				{
					ack_state = 1;
					for(int x = 0; x < resp_length + 2; x++)
						tx_packet[x] = resp_crc_ok[x];
				}
				else
				{
					ack_state = -1;
					for(int x = 0; x < resp_length + 2; x++)
						tx_packet[x] = resp_crc_fail[x];
				}
				
				if(resp_length > 0)
				{
					current_mode = rm_tx2rx;
					NRF_RADIO->PACKETPTR = (uint32_t)tx_packet;
					NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
					NRF_RADIO->TASKS_TXEN = 1;
					while(NRF_RADIO->STATE == st_radio_disabled) ;
					NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
				}
				else
				{
					current_mode = rm_rx2rx;
					NRF_RADIO->TASKS_START = 1; //in case if we are in RXIDLE
				}
			}
			else
			{
				need_call_rx_cplt = 1;
				current_mode = rm_rx2rx;
				NRF_RADIO->TASKS_START = 1; //in case if we are in RXIDLE
			}
		}
	}
	if(!handled)
	{
		rf_busy = 0;
		rf_clear_events();
	}
	if(need_call_tx_cplt && rf_tx_irq != NULL) rf_tx_irq();
	if(need_call_rx_cplt && rf_rx_irq != NULL) rf_rx_irq();
}

int rf_get_ack_state()
{
	return ack_state;
}

int rf_is_busy()
{
//	return 0;
	return rf_busy;
}
