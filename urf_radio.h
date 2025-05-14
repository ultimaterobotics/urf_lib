#include <stdint.h>

typedef struct {
    union {
        uint32_t reg;
        struct {
			unsigned length_bitsz : 4;
			unsigned : 4;
			unsigned S0_bytesz : 1; //only 0 or 1 bytes
			unsigned : 7;
			unsigned S1_bitsz : 4;
			unsigned S1_include : 1;
			unsigned : 3;
			unsigned preamble_length : 1;
			unsigned : 7;
        };
    };
} NRF52_PCNF0_REG;


typedef struct {
    union {
        uint32_t reg;
        struct {
			unsigned max_length : 8;
			unsigned added_length : 8;
			unsigned addr_len : 3;
			unsigned : 5;
			unsigned endian : 1;
			unsigned whitening : 1;
			unsigned : 6;
        };
    };
} NRF52_PCNF1_REG;

enum
{
	st_radio_disabled = 0,
	st_radio_rxru = 1,
	st_radio_rxidle,
	st_radio_rx,
	st_radio_rxdisable,
	st_radio_txru = 9,
	st_radio_txidle,
	st_radio_tx,
	st_radio_txdisable
};

enum
{
	rm_rx_end = 1,
	rm_tx_end,
	rm_rx2tx,
	rm_tx2rx,
	rm_rx2rx
};

void rf_disable();
void rf_init(int channel, int speed, int crc_len);
void rf_init_ext(int channel, int speed, int crc_len, int white_en, int s1_sz, int added_length, int max_length);
void rf_mode_rx_only();
void rf_mode_tx_only();
void rf_mode_tx_then_rx();
void rf_send(uint8_t *pack, int length); //byte 0 of the packet can be anything, byte 1 defines payload length
void rf_listen();
void rf_send_and_listen(uint8_t *pack, int length); //byte 0 of the packet can be anything, byte 1 defines payload length

void rf_autorespond_on(uint8_t *pack_crc_ok, uint8_t *pack_crc_fail, int resp_pack_length);
void rf_autorespond_off();

void rf_override_irq(void (*new_irq));

void rf_attach_rx_irq(void (*rx_irq));
void rf_dettach_rx_irq();

void rf_attach_tx_irq(void (*tx_irq));
void rf_dettach_tx_irq();

int rf_has_new_packet();
uint32_t rf_get_packet(uint8_t *pack);
int rf_get_ack_state();

int rf_is_busy();
void rf_clear_events();