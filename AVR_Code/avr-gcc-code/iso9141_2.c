/*
 * ISO9141_2_low_level.c
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#include <stdbool.h>
#include <inttypes.h>

#include <avr/io.h>
#include <util/delay.h>

#include "iso9141_2_uart.h"

#define K_OUT PD3 /* arduino - 18, mega PD3 */
#define K_IN  PD2 /* arduino - 19, mega PD2 */

static void rx_off(void)
{
	UCSR1B &= ~(_BV(RXEN1));  //disable UART RX
}

static void rx_on(void)
{
	ISO9141_2_UART_init();
}

static void tx_off(void)
{
	   UCSR1B &= ~(_BV(TXEN1));  //disable UART TX
	   _delay_ms(20);                 //allow time for buffers to flush
}

bool ISO9141_2_Init(void)
{
	/* set tx and rx pins */
	DDRD |= (1 << K_OUT);
	DDRD &= ~(1 << K_IN);

	return true;
}

void ISO9141_2_WriteByte(uint8_t byte)
{
	/*
	  serial_rx_off();
	  Serial1.write(b);
	  delay(10);    // ISO requires 5-20 ms delay between bytes.
	  serial_rx_on();
	 */
	rx_off();
	ISO9141_2_WriteByte(byte);
	_delay_ms(10);
	rx_on();
}

uint8_t	ISO9141_2_ReadByte(void)
{
	/*
	  int b;
	  byte t=0;
	  while(t!=125  && (b=Serial1.read())==-1) {
	    delay(1);
	    t++;
	  }
	  if (t>=125) {
	    b = 0;
	  }
	  return b;
	*/

	/* NB! - Should check implementation of Serial.read to see if my implementation of receive is the same */

	uint8_t b;
	uint8_t t=0;

	while(t!=125  && (b=ISO9141_2_UART_Receive())==-1)
	{
		_delay_ms(1);
		t++;
	}

	if (t>=125)
	{
		b = 0;
	}

	return b;
}

uint8_t	ISO9141_2_WriteData(uint8_t *data, uint8_t len)
{

}

uint8_t	ISO9141_2_ReadData(uint8_t *data, uint8_t len)
{

}

