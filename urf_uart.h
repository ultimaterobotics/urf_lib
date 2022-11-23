#include <stdint.h>

void uart_init(int pin_TX, int pin_RX, int speed);
void uart_send(uint8_t *buf, int length);
int uart_send_remains();
uint32_t uart_get_rx_position();
uint8_t *uart_get_rx_buf();
uint32_t uart_get_rx_buf_length();
void uprintf( const char * format, ... );
void uhex_print(volatile uint8_t *buf, int len, int spaced);
void uart_shutdown();
