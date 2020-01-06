#include <string.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "UART.h"

void ProcessCommands(uint8_t cmd);
void K_LineReceiver(uint8_t data);

int main(void)
{
	/* communication port with PC */
	UART_Init(UART_0);
	UART_SetRxCallback(UART_0,ProcessCommands);
	UART_SetBaudRate(UART_0,57600);
	UART_TransmitMultibyte(UART_0,"UART_0 initialized\r\n",strlen("UART_0 initialized\r\n"));

	/* K Line interface */
	UART_Init(UART_3);
	UART_SetRxCallback(UART_3,K_LineReceiver);
	UART_SetBaudRate(UART_3,12700);
	UART_TransmitMultibyte(UART_3,"UART_3 initialized\r\n",strlen("UART_3 initialized\r\n"));

	sei();

	while(42)
	{
		UART_TransmitMultibyte(UART_0,(uint8_t *)"UART0:BOGDAJ, VRAKZEMI\r\n",strlen("UART2:BOGDAJ, VRAKZEMI\r\n"));
		_delay_ms(1000);
	}

	return 0;
}

void ProcessCommands(uint8_t cmd)
{
	UART_Transmit(UART_0,cmd);
}

void K_LineReceiver(uint8_t data)
{
	UART_Transmit(UART_3,data);
}
