/*
 * main.c
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */
#include <stdbool.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "pc_com_uart.h"

int main(void)
{
	char dgb_str[64];
	int8_t ecu_initialized = -1;

	PC_COM_UART_init();
	PC_COM_UART_Puts("VOLVO 850 DIAG STARTED\r\n");

	while(1)
	{
		if (ecu_initialized == -1)
		{
			PC_COM_UART_Puts("TRYING TO SETUP ECU\r\n");
			ecu_initialized = ISO9414_2_Init();
			sprintf(dgb_str,"ecu_initialized = %d\r\n",ecu_initialized);
			PC_COM_UART_Puts(dgb_str);
			if (ecu_initialized == 0)
			{
				PC_COM_UART_Puts("ECU INITI OK\r\n");
			}
		}
		_delay_ms(1000);
	}

	return 0;
}
