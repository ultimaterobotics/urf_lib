#include <stdio.h>
#include <stdarg.h>

#include "urf_uart.h"
#include "nrf.h"

uint8_t uart_buf[258];
uint32_t uart_buf_length = 256;
volatile uint8_t uart_buf_pos = 0;

void uart_init(int pin_TX, int pin_RX, int speed)
{
	NRF_UART0->PSELRTS = 0xFFFFFFFF;
	NRF_UART0->PSELCTS = 0xFFFFFFFF;
	NRF_UART0->PSELTXD = pin_TX;
	NRF_UART0->PSELRXD = pin_RX;
	
	NRF_GPIO->DIRSET = 1<<pin_TX;
//	nrf_gpio_cfg_output(pin_TX);
//	nrf_gpio_cfg_input(pin_RX, NRF_GPIO_PIN_NOPULL);

	if(speed == 1200) //list of standard speeds with best approximations
		NRF_UART0->BAUDRATE = 0x0004F000;
	else if(speed == 2400)
		NRF_UART0->BAUDRATE = 0x0009D000;
	else if(speed == 4800)
		NRF_UART0->BAUDRATE = 0x0013B000;
	else if(speed == 9600)
		NRF_UART0->BAUDRATE = 0x00275000;
	else if(speed == 14400)
		NRF_UART0->BAUDRATE = 0x003B0000;
	else if(speed == 19200)
		NRF_UART0->BAUDRATE = 0x004EA000;
	else if(speed == 28800)
		NRF_UART0->BAUDRATE = 0x0075F000;
	else if(speed == 38400)
		NRF_UART0->BAUDRATE = 0x009D5000;
	else if(speed == 57600)
		NRF_UART0->BAUDRATE = 0x00EBF000;
	else if(speed == 76800)
		NRF_UART0->BAUDRATE = 0x013A9000;
	else if(speed == 115200)
		NRF_UART0->BAUDRATE = 0x01D7E000;
	else if(speed == 230400)
		NRF_UART0->BAUDRATE = 0x03AFB000;
	else if(speed == 250000)
		NRF_UART0->BAUDRATE = 0x04000000;
	else if(speed == 460800)
		NRF_UART0->BAUDRATE = 0x075F7000;
	else if(speed == 921600)
		NRF_UART0->BAUDRATE = 0x0EBEDFA4;
	else if(speed == 1000000)
		NRF_UART0->BAUDRATE = 0x10000000;
	else //not from standard list - use simple rounding
	{
		NRF_UART0->BAUDRATE = 0x10000000 / 1000000 * speed;
	}

	NRF_UART0->CONFIG = 0; //no hardware control, no parity
	NRF_UART0->INTENSET = 0b10000100; //TXDRDY, RXDRDY
	NRF_UART0->ENABLE = 0b0100;

	NVIC_EnableIRQ(UART0_IRQn);
	
	NRF_UART0->TASKS_STARTRX = 1;
}

uint8_t send_buf[1024];
volatile uint16_t send_start = 0;
volatile uint16_t send_end = 0;
volatile uint16_t send_complete = 1;

void uart_send(uint8_t *buf, int length)
{
	if(length > 255) return;
	uint16_t pos = send_end;
	for(int x = 0; x < length; x++)
	{
		send_buf[pos++] = buf[x];
		pos &= 0x3FF;
		if(pos == send_start)
		{
			send_start++;
			send_start &= 0x3FF;
		}
	}
	send_end = pos;

	if(send_complete)
	{
		NRF_UART0->TXD = send_buf[send_start];
		send_start++;
		NRF_UART0->TASKS_STARTTX = 1;
		send_complete = 0;
	}
}

int uart_send_remains()
{
	if(send_end < send_start) return send_end - send_start + 1024;
	else return send_end - send_start;
}

void UART0_IRQHandler()
{
    if(NRF_UART0->EVENTS_TXDRDY) 
	{
		NRF_UART0->EVENTS_TXDRDY = 0;
		if(send_end != send_start)
		{
			NRF_UART0->TXD = send_buf[send_start++];
			send_start &= 0x3FF;
		}
		else
		{
			send_complete = 1;
		}
    }  
	else if(NRF_UART0->EVENTS_RXDRDY) 
	{
		NRF_UART0->EVENTS_RXDRDY = 0;
		uart_buf[uart_buf_pos++] = NRF_UART0->RXD;
    }

}

uint32_t uart_get_rx_position()
{
	return uart_buf_pos;
}
uint8_t *uart_get_rx_buf()
{
	return uart_buf;
}
uint32_t uart_get_rx_buf_length()
{
	return uart_buf_length;
}

uint8_t umsg[256];

void uprintf( const char * format, ... )
{
	va_list args;

	va_start(args, format);
	int ulen = vsnprintf(umsg, 255, format, args);
	va_end(args);
	uart_send(umsg, ulen);
}
