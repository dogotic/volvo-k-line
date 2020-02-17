/*
 * main.c
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "pc_com_uart.h"

int main(void)
{
	PC_COM_UART_init();
	PC_COM_UART_Puts("VOLVO 850 DIAG STARTED");

	while(1)
	{

	}

	return 0;
}
