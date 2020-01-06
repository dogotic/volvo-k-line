/*
 * UART.c
 *
 *  Created on: Jan 5, 2020
 *      Author: kaktus
 */

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "UART.h"

void (*rx_handlers[4]) (uint8_t data);

void UART_Init(UART_Number number)
{
	switch(number)
	{
		case UART_0:
			UCSR0B|=(1<<TXEN0); //enable TX
			UCSR0B|=(1<<RXEN0); //enable RX
			UCSR0B|=(1<<RXCIE0); //RX complete interrupt
			UCSR0C|=(1<<UCSZ01)|(1<<UCSZ01); // no parity, 1 stop bit, 8-bit data
		break;

		case UART_1:
			UCSR1B|=(1<<TXEN1); //enable TX
			UCSR1B|=(1<<RXEN1); //enable RX
			UCSR1B|=(1<<RXCIE1); //RX complete interrupt
			UCSR1C|=(1<<UCSZ11)|(1<<UCSZ11); // no parity, 1 stop bit, 8-bit data
		break;

		case UART_2:
			UCSR2B|=(1<<TXEN2); //enable TX
			UCSR2B|=(1<<RXEN2); //enable RX
			UCSR2B|=(1<<RXCIE2); //RX complete interrupt
			UCSR2C|=(1<<UCSZ21)|(1<<UCSZ21); // no parity, 1 stop bit, 8-bit data
		break;

		case UART_3:
			UCSR3B|=(1<<TXEN3); //enable TX
			UCSR3B|=(1<<RXEN3); //enable RX
			UCSR3B|=(1<<RXCIE3); //RX complete interrupt
			UCSR3C|=(1<<UCSZ31)|(1<<UCSZ31); // no parity, 1 stop bit, 8-bit data
		break;
	}
}

void UART_SetBaudRate(UART_Number number, uint32_t BaudRate)
{
	uint16_t ubrr_val = (((F_CPU/(BaudRate*16UL)))-1);

	switch(number)
	{
		case UART_0:
			UBRR0 = ubrr_val;
		break;

		case UART_1:
			UBRR1 = ubrr_val;
		break;

		case UART_2:
			UBRR2 = ubrr_val;
		break;

		case UART_3:
			UBRR3 = ubrr_val;
		break;
	}
}

void UART_Transmit(UART_Number number, uint8_t data)
{
	switch(number)
	{
		case UART_0:
			// send a single character via USART
			while(!(UCSR0A&(1<<UDRE0))){}; //wait while previous byte is completed
			UDR0 = data; // Transmit data
		break;

		case UART_1:
			// send a single character via USART
			while(!(UCSR1A&(1<<UDRE1))){}; //wait while previous byte is completed
			UDR1 = data; // Transmit data
		break;

		case UART_2:
			// send a single character via USART
			while(!(UCSR2A&(1<<UDRE2))){}; //wait while previous byte is completed
			UDR2 = data; // Transmit data
		break;

		case UART_3:
			// send a single character via USART
			while(!(UCSR3A&(1<<UDRE3))){}; //wait while previous byte is completed
			UDR3 = data; // Transmit data
		break;
	}
}

void UART_TransmitMultibyte(UART_Number number, uint8_t *data, uint8_t len)
{
	int i = 0;
	for (i=0; i<len; i++)
	{
		UART_Transmit(number,data[i]);
	}
}

void UART_SetRxCallback(UART_Number number, void (*fn)(uint8_t data))
{
	switch(number)
	{
		case UART_0:
			rx_handlers[0] = fn;
		break;

		case UART_1:
			rx_handlers[1] = fn;
		break;

		case UART_2:
			rx_handlers[2] = fn;
		break;

		case UART_3:
			rx_handlers[3] = fn;
		break;
	}
}

ISR(USART0_RX_vect)
{
	rx_handlers[0](UDR0);
}

ISR(USART1_RX_vect)
{
	rx_handlers[1](UDR1);
}

ISR(USART2_RX_vect)
{
	rx_handlers[2](UDR2);
}

ISR(USART3_RX_vect)
{
	rx_handlers[3](UDR3);
}

