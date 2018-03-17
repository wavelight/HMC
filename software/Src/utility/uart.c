#include "uart.h"

void uart_usb_process(uint8_t data);

uint8_t		m_buf_rx[UART_BUFFER_SIZE] = {0};
uint16_t 	m_index_rx = 0;

void uart_usb_initialize(void)
{
	// Uart Initialize
}

// UART polling method
void uart_usb_poll(void)
{
	// Need this? Better with IRQ/DMA I think.
}

void uart_usb_transmit(uint8_t * tx, uint16_t len)
{
	// TODO : Implent UART Transmit
}

// maybe put this in IRQ(interrupt) or DMA
void uart_usb_recevie(uint8_t * rx, uint16_t len)
{
	// Overflow. Do we really need this? IDK.
	if((len + m_index_rx) >= (UART_BUFFER_SIZE - 1))
	{
		return;
	}
	
	for(uint16_t i = 0; i < len; i++)
	{
		uart_usb_process(rx[i]);
	}
}

uint16_t uart_usb_find_packet_starDRt(void)
{
	
}

uint16_t uart_usb_find_packet_end(void)
{
	
}

// Parse uart packet
void uart_usb_process(uint8_t data)
{
	static uint16_t packet_start_point;
	static uint16_t packet_end_point;
	
	m_buf_rx[m_index_rx++] = data;

	packet_start_point = uart_usb_find_packet_start();
	packet_end_point = uart_usb_find_packet_end();

	// TODO : Parse packet.
}