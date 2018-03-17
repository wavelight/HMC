#include "uart.h"

void uart_usb_process(uint8_t data);

uint8_t		m_buf_rx[UART_USB_BUFFER_SIZE] = {0};
uint16_t 	m_index_rx = 0;

void uart_usb_initialize(void)
{
	// TODO : Uart Initialize
}

// UART polling method
void uart_usb_poll(void)
{
	// TODO : Need this? Better with IRQ/DMA I think.
}

void uart_usb_transmit(uint8_t * tx, uint16_t len)
{
	// TODO : Implent UART Transmit
}

// maybe put this in IRQ(interrupt) or DMA
void uart_usb_recevie(uint8_t * rx, uint16_t len)
{
	// TODO : Overflow. Do we really need this? IDK.
	// Just clear buffer maybe?
	if((len + m_index_rx) >= (UART_USB_BUFFER_SIZE - 1))
	{
		return;
	}
	
	for(uint16_t i = 0; i < len; i++)
	{
		uart_usb_process(rx[i]);
	}
}

uint16_t uart_usb_find_packet_start(void)
{
	uint16_t startPoint;

	for(uint16_t i = 0; i < UART_USB_BUFFER_SIZE; i++)
	{
		if(m_buf_rx[i] == UART_USB_PACKET_START_1)
		{
			if(m_buf_rx[i + 1] == UART_USB_PACKET_START_2)
			{
				return i;
			}
		}
	}

	return UART_USB_PACKET_START_NULL;
}

uint16_t uart_usb_find_packet_end(void)
{
	for(uint16_t i = 0; i < UART_USB_BUFFER_SIZE; i++)
	{
		if(m_buf_rx[i] == UART_USB_PACKET_END_1)
		{
			if(m_buf_rx[i + 1] == UART_USB_PACKET_END_2)
			{
				return i + 1;
			}
		}
	}

	return UART_USB_PACKET_END_NULL;
}

// Parse uart packet
void uart_usb_process(uint8_t data)
{
	static uint16_t packet_start_point;
	static uint16_t packet_end_point;
	
	// What if index overflow?
	m_buf_rx[m_index_rx++] = data;

	packet_start_point = uart_usb_find_packet_start();
	if(packet_start_point == UART_USB_PACKET_START_NULL)
	{
		return;
	}

	packet_end_point = uart_usb_find_packet_end();
	if(packet_end_point == UART_USB_PACKET_END_NULL)
	{
		return;
	}

	// Packet end point cannot be placed before the start point. packet not valid situation.
	if(packet_end_point < packet_start_point)
	{
		// More readable?
		// TODO : Check logic here
		memcpy(&m_buf_rx[0], &m_buf_rx[packet_end_point + 1], (UART_USB_BUFFER_SIZE - packet_end_point));
		m_index_rx = m_index_rx - packet_end_point;
		return;
	}

	// TODO : Parse packet.
}