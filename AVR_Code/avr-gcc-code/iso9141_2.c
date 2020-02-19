/*
 * ISO9141_2_low_level.c
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include <avr/io.h>
#include <util/delay.h>

#include "iso9141_2.h"
#include "iso9141_2_uart.h"
#include "pc_com_uart.h"

#define K_OUT PD3 /* arduino - 18, mega PD3 */
#define K_IN  PD2 /* arduino - 19, mega PD2 */

static void serial_rx_off(void)
{
	UCSR1B &= ~(_BV(RXEN1));  //disable UART RX
}

static void serial_rx_on(void)
{
	ISO9141_2_UART_init();
}

static void serial_tx_off(void)
{
	UCSR1B &= ~(_BV(TXEN1));  //disable UART TX
	_delay_ms(20);                 //allow time for buffers to flush
}

void ISO9141_2_WriteByte(uint8_t byte)
{
	/*
	 serial_rx_off();
	 Serial1.write(b);
	 delay(10);    // ISO requires 5-20 ms delay between bytes.
	 serial_rx_on();
	 */
	serial_rx_off();
	ISO9141_2_UART_Transmit(byte);
	_delay_ms(10);
	serial_rx_on();
}

uint8_t ISO9141_2_ReadByte(void)
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

	int8_t b;
	uint8_t t = 0;

	while (t != 125 && (b = ISO9141_2_UART_Receive()) == -1)
	{
		_delay_ms(1);
		t++;
	}

	if (t >= 125)
	{
		b = 0;
	}

	return b;
}

uint8_t ISO9141_2_WriteData(uint8_t *data, uint8_t len)
{
	uint8_t i, n;
	uint8_t buf[20];

	// ISO header
	buf[0] = 0x68;
	buf[1] = 0x6A;    // 0x68 0x6A is an OBD-II request
	buf[2] = 0xF1;    // our requesters address (off-board tool)
	// append message
	for (i = 0; i < len; i++)
		buf[i + 3] = data[i];

	// calculate checksum
	i += 3;
	buf[i] = ISO9141_2_Checksum(buf, i);

	// send char one by one
	n = i + 1;
	for (i = 0; i < n; i++)
	{
		ISO9141_2_WriteByte(buf[i]);
	}

	return 0;
}

uint8_t ISO9141_2_ReadData(uint8_t *data, uint8_t len)
{
	uint8_t i;
	uint8_t buf[20];

	// header 3 bytes: [80+datalen] [destination=f1] [source=01]
	// data 1+1+len bytes: [40+cmd0] [cmd1] [result0]
	// checksum 1 bytes: [sum(header)+sum(data)]

	for (i = 0; i < 3 + 1 + 1 + 1 + len; i++)
		buf[i] = ISO9141_2_ReadByte();

	// test, skip header comparison
	// ignore failure for the moment (0x7f)
	// ignore crc for the moment

	// we send only one command, so result start at buf[4] Actually, result starts at buf[5], buf[4] is pid requested...
	memcpy(data, buf + 5, len);

	_delay_ms(55);    //guarantee 55 ms pause between requests

	return len;
}

uint8_t ISO9141_2_Checksum(uint8_t *data, uint8_t len)
{
	uint8_t i;
	uint8_t crc;

	crc = 0;
	for (i = 0; i < len; i++)
		crc = crc + data[i];

	return crc;
}

int8_t ISO9414_2_Init(void)
{
	uint8_t b;
	uint8_t kw1;
	uint8_t kw2;
	uint8_t mask;
	uint8_t i;
	char dbg_string[64];

	/* set tx and rx pins */
	DDRD |= (1 << K_OUT);
	DDRD &= ~(1 << K_IN);

	serial_tx_off();
	serial_rx_off();
	_delay_ms(3000);

	PORTD |= (1 << K_OUT);
	_delay_ms(300);

  	// send 0x10 at 5 bauds

  	// start bit	
  	PORTD &= ~(1 << K_OUT);
  	_delay_ms(200);

  	// data
  	b=0x10;
	for (mask = 0x01; mask; mask <<= 1)
	{
		if (b & mask) // choose bit
		{
			//digitalWrite(K_OUT, HIGH); // send 1
			PORTD |= (1 << K_OUT);
		}
		else
		{
			// digitalWrite(K_OUT, LOW); // send 0
			PORTD &= ~(1 << K_OUT);
		}
		_delay_ms(200);
	}

	// stop bit + 60 ms delay
	PORTD |= (1 << K_OUT);
	_delay_ms(260);


	// switch now to 12700 bauds
	ISO9141_2_UART_init();


  	// wait for 0x55 from the ECU (up to 300ms)
  	//since our time out for reading is 125ms, we will try it three times
  	for(i=0; i<3; i++) 
  	{
    	b=ISO9141_2_ReadByte(); 
    	// Serial.print("ECU ACKNOWLEDGE : ");
    	// Serial.println(b, HEX);
    	sprintf(dbg_string,"ECU ACKNOWLEDGE 0x%02X\r\n",b);
  		PC_COM_UART_Puts(dbg_string);

    	if(b==0x55)
      		break;
  	}

  	if(b!=0x55)
  	{
		PC_COM_UART_Puts("ACK FAILED\r\n");
    	return -1;  
  	}

  	// wait for kw1 and kw2
  	kw1=ISO9141_2_ReadByte();
  	// Serial.print("kw1 : ");
  	// Serial.println(kw1,HEX);
  	sprintf(dbg_string,"KW1 : 0x%02X\r\n",kw1);
  	PC_COM_UART_Puts(dbg_string);

	kw2=ISO9141_2_ReadByte();
  	// Serial.print("kw2 : ");
  	// Serial.println(kw2,HEX);  
  	sprintf(dbg_string,"KW2 : 0x%02X\r\n",kw2);
  	PC_COM_UART_Puts(dbg_string);  	
  	_delay_ms(250);

	// sent ~kw2 (invert of last keyword)
	ISO9141_2_WriteByte(~kw2);
	_delay_ms(250);

	// ECU answer by 0xAB
	b=ISO9141_2_ReadByte();
	sprintf(dbg_string,"ECU ANSWER: 0x%02X\r\n",b);
	PC_COM_UART_Puts(dbg_string);	

	if(b != 0xAB)
	{
		return -1;
	}

	// init OK!
	return 0;

}

