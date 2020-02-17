/*
 * iso9141_2_uart.h
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#include <inttypes.h>
#include <avr/io.h>

void ISO9141_2_UART_init(void)
{
	/* baud rate is fixed to 12700 */

	// calc UBRR for a given baudRate
	// UBRR = (FOSC / (16*BAUDRATE)) - 1
	uint16_t ubrr = 77;

	// set UBRR
	UBRR1H = (uint8_t)(ubrr>>8);
	UBRR1L = (uint8_t)ubrr;

	// USART SETUP
	// Enable receiver
	// Enable transmitter
	// Enable receiver interrupts
	UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);

	// Frame format
	// 8 data bits
	// 1 stop bit
	// no parity bits
	UCSR1C = (3<<UCSZ10);

}

void ISO9141_2_UART_Transmit(uint8_t data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) ) ;
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

int8_t ISO9141_2_UART_Receive(void)
{
	/* Wait for empty transmit buffer */
    if (!(UCSR1A & (1<<RXC1)))
    {
    	return -1;
    }
    else
    {
	/* return data from the USART receive buffer */
    	return UDR1;
    }
}
