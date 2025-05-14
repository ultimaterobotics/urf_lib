#include <stdio.h>
#include <stdarg.h>

#include "urf_uart.h"
#include "nrf.h"

uint8_t uart_buf_rx[290]; //+32 bytes due to batch size
uint32_t uart_rx_length = 256;
volatile uint8_t uart_rx_pos = 0;

uint8_t uart_buf_tx[530];
volatile uint16_t uart_transaction_end = 0;
volatile uint16_t uart_scheduled_end = 0;
volatile int send_complete = 1;
int uart_tx_length = 512;
int send_tx_size = 64;
int uart_inited = 0;
int rx_batch_size = 1;

void uart_init(int pin_TX, int pin_RX, int speed)
{
	NRF_GPIO->PIN_CNF[pin_RX] = 0;
	NRF_GPIO->PIN_CNF[pin_TX] = 1;
	
	NRF_UARTE0->PSEL.RTS = 0xFFFFFFFF;
	NRF_UARTE0->PSEL.CTS = 0xFFFFFFFF;
	NRF_UARTE0->PSEL.TXD = pin_TX;
	NRF_UARTE0->PSEL.RXD = pin_RX;
	
//	nrf_gpio_cfg_output(pin_TX);
//	nrf_gpio_cfg_input(pin_RX, NRF_GPIO_PIN_NOPULL);

	if(speed == 1200) //list of standard speeds with best approximations
		NRF_UARTE0->BAUDRATE = 0x0004F000;
	else if(speed == 2400)
		NRF_UARTE0->BAUDRATE = 0x0009D000;
	else if(speed == 4800)
		NRF_UARTE0->BAUDRATE = 0x0013B000;
	else if(speed == 9600)
		NRF_UARTE0->BAUDRATE = 0x00275000;
	else if(speed == 14400)
		NRF_UARTE0->BAUDRATE = 0x003B0000;
	else if(speed == 19200)
		NRF_UARTE0->BAUDRATE = 0x004EA000;
	else if(speed == 28800)
		NRF_UARTE0->BAUDRATE = 0x0075F000;
	else if(speed == 38400)
		NRF_UARTE0->BAUDRATE = 0x009D5000;
	else if(speed == 57600)
		NRF_UARTE0->BAUDRATE = 0x00EBF000;
	else if(speed == 76800)
		NRF_UARTE0->BAUDRATE = 0x013A9000;
	else if(speed == 115200)
		NRF_UARTE0->BAUDRATE = 0x01D7E000;
	else if(speed == 230400)
		NRF_UARTE0->BAUDRATE = 0x03AFB000;
	else if(speed == 250000)
		NRF_UARTE0->BAUDRATE = 0x04000000;
	else if(speed == 460800)
		NRF_UARTE0->BAUDRATE = 0x075F7000;
	else if(speed == 921600)
		NRF_UARTE0->BAUDRATE = 0x0EBEDFA4;
	else if(speed == 1000000)
		NRF_UARTE0->BAUDRATE = 0x10000000;
	else if(speed == 2000000) //experimental
		NRF_UARTE0->BAUDRATE = 0x20000000;
	else //not from standard list - use simple rounding
	{
		NRF_UARTE0->BAUDRATE = 0x10000000 / 1000000 * speed;
	}

	NRF_UARTE0->CONFIG = 0; //no hardware control, no parity
	NRF_UARTE0->INTEN = (1<<4) | (1<<8) | (1<<17); //ENDRX, ENDTX, RXTO
//	NRF_UARTE0->INTEN = (1<<8); //ENDTX
	NRF_UARTE0->ENABLE = 8;

	NVIC_EnableIRQ(UARTE0_UART0_IRQn);//52832
	NVIC_EnableIRQ(UART0_IRQn);//52810
	
	NRF_UARTE0->RXD.PTR = (uint32_t)uart_buf_rx;
	NRF_UARTE0->RXD.MAXCNT = rx_batch_size;
	NRF_UARTE0->TASKS_STARTRX = 1;
	NRF_UARTE0->SHORTS = 0b100000; //ENDRX_STARTRX
	uart_inited = 1;
}
void uart_shutdown()
{
	NVIC_DisableIRQ(UARTE0_UART0_IRQn);//52832
	NVIC_DisableIRQ(UART0_IRQn);//52810
	NRF_UARTE0->ENABLE = 0;
	uart_inited = 0;
}


void uart_update_tx()
{
	if(uart_transaction_end == uart_scheduled_end) return;
	NRF_UARTE0->TXD.PTR = (uint32_t)uart_buf_tx + uart_transaction_end;
	int transf_len = uart_scheduled_end - uart_transaction_end;
	if(transf_len < 0)
	{
		transf_len = uart_tx_length - uart_transaction_end;
		if(transf_len > send_tx_size) transf_len = send_tx_size;
		NRF_UARTE0->TXD.MAXCNT = transf_len;
		uart_transaction_end += transf_len;
		if(uart_transaction_end >= uart_tx_length) uart_transaction_end = 0;
	}
	else
	{
		transf_len = uart_scheduled_end - uart_transaction_end;
		if(transf_len > send_tx_size) transf_len = send_tx_size;
		NRF_UARTE0->TXD.MAXCNT = transf_len;
		uart_transaction_end += transf_len;
		if(uart_transaction_end >= uart_tx_length) uart_transaction_end = 0;
	}
	send_complete = 0;
	NRF_UARTE0->TASKS_STARTTX = 1;
}

void uart_send(uint8_t *buf, int length)
{
	if(!uart_inited) return;
	if(length > 255) return;
	uint16_t pos = uart_scheduled_end;
	for(int x = 0; x < length; x++)
	{
		uart_buf_tx[pos++] = buf[x];
		if(pos >= uart_tx_length) pos = 0;
	}
	uart_scheduled_end = pos;

	if(send_complete)
	{
		uart_update_tx();
	}
}

int uart_send_remains()
{
	return 0; //not implemented
//	if(uart_scheduled_end < uart_transaction_end) return send_end - send_start + send_buf_length;
//	else return send_end - send_start;
}

void uart_irq_handler()
{
    if(NRF_UARTE0->EVENTS_ENDTX) 
	{
		NRF_UARTE0->EVENTS_ENDTX = 0;
		if(uart_transaction_end != uart_scheduled_end)
		{			
			uart_update_tx();
		}
		else
		{
			send_complete = 1;
		}
    }  
	else if(NRF_UARTE0->EVENTS_ENDRX) 
	{
		NRF_UARTE0->EVENTS_ENDRX = 0;
		uart_rx_pos += NRF_UARTE0->RXD.AMOUNT;
		if(uart_rx_pos >= uart_rx_length)
		{
			for(int n = uart_rx_length; n < uart_rx_pos; n++)
				uart_buf_rx[n-uart_rx_length] = uart_buf_rx[n];
			uart_rx_pos -= uart_rx_length;
		}
		NRF_UARTE0->RXD.PTR = (uint32_t)uart_buf_rx + uart_rx_pos;
		NRF_UARTE0->TASKS_STARTRX = 1;
    }
	else if(NRF_UARTE0->EVENTS_RXTO) 
	{
		NRF_UARTE0->EVENTS_RXTO = 0;
		uart_rx_pos += NRF_UARTE0->RXD.AMOUNT;
		if(uart_rx_pos >= uart_rx_length)
		{
			for(int n = uart_rx_length; n < uart_rx_pos; n++)
				uart_buf_rx[n-uart_rx_length] = uart_buf_rx[n];
			uart_rx_pos -= uart_rx_length;
		}
		NRF_UARTE0->RXD.PTR = (uint32_t)uart_buf_rx + uart_rx_pos;
		NRF_UARTE0->TASKS_STARTRX = 1;
    }

/*	NRF_UARTE0->EVENTS_CTS = 0;
	NRF_UARTE0->EVENTS_NCTS = 0;
	NRF_UARTE0->EVENTS_ENDRX = 0;
	NRF_UARTE0->EVENTS_ENDTX = 0;
	NRF_UARTE0->EVENTS_ERROR = 0;
	NRF_UARTE0->EVENTS_RXTO = 0;
	NRF_UARTE0->EVENTS_RXSTARTED = 0;
	NRF_UARTE0->EVENTS_TXSTARTED = 0;
	NRF_UARTE0->EVENTS_TXSTOPPED = 0;*/
}

void UARTE0_UART0_IRQHandler() //52832
{
	uart_irq_handler();
}

void UARTE0_IRQHandler() //52810
{
	uart_irq_handler();
}

uint32_t uart_get_rx_position()
{
	return uart_rx_pos;
}
uint8_t *uart_get_rx_buf()
{
	return uart_buf_rx;
}
uint32_t uart_get_rx_buf_length()
{
	return uart_rx_length;
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
void uhex_print(volatile uint8_t *buf, int len, int spaced)
{
	int pos = 0;
	for(int n = 0; n < len; n++)
	{
		int bf = buf[n];
		int hb = bf>>4;
		int lb = bf&0x0F;
		int lv, hv;
		if(lb < 10) lv = '0' + lb;
		else lv = 'A' + lb-10;
		if(hb < 10) hv = '0' + hb;
		else hv = 'A' + hb-10;
		umsg[pos++] = hv;
		umsg[pos++] = lv;
		if(spaced) umsg[pos++] = ' ';
		if(pos > 250) break;
	}
	uart_send(umsg, pos);
}
