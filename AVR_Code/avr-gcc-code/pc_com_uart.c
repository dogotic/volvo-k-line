/*
 * pc_com_uart.c
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#include <inttypes.h>
#include <avr/io.h>

void PC_COM_UART_init(void)
{
	/* baudrate is fixed to 115.200k */

	// calc UBRR for a given baudRate
	// UBRR = (FOSC / (16*BAUDRATE)) - 1
	uint16_t ubrr = 8;

	// set UBRR
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;

	// USART SETUP
	// Enable receiver
	// Enable transmitter
	// Enable receiver interrupts
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);

	// Frame format
	// 8 data bits
	// 1 stop bit
	// no parity bits
	UCSR0C = (3<<UCSZ00);
}

void PC_COM_UART_Transmit(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) ) ;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

uint8_t PC_COM_UART_Receive(void)
{
	/* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<RXC0)));
	/* return data from the USART receive buffer */
    return UDR0;
}

// USART send string handler
void PC_COM_UART_Puts(const char *string)
{
	while(*string)
	{
		PC_COM_UART_Transmit(*string);
		++string;
	}
}
