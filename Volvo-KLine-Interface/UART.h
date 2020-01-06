/*
 * UART.h
 *
 *  Created on: Jan 5, 2020
 *      Author: kaktus
 */

#ifndef UART_H_
#define UART_H_

typedef enum UART_Number
{
	UART_0,
	UART_1,
	UART_2,
	UART_3,
} UART_Number;

void UART_Init(UART_Number number);
void UART_SetBaudRate(UART_Number number, uint32_t BaudRate);
void UART_Transmit(UART_Number number, uint8_t data);
void UART_TransmitMultibyte(UART_Number number, uint8_t *data, uint8_t len);
void UART_SetRxCallback(UART_Number number, void (*fn)(uint8_t data));

#endif /* UART_H_ */
