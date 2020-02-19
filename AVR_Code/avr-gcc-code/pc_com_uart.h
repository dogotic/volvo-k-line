/*
 * pc_comm_uart.h
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#ifndef PC_COM_UART_H_
#define PC_COM_UART_H_

void PC_COM_UART_init(void);
void PC_COM_UART_Transmit(uint8_t data);
uint8_t PC_COM_UART_Receive(void);
void PC_COM_UART_Puts(const char *string);
#endif /* PC_COM_UART_H_ */
