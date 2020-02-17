/*
 * iso9141_2_uart.h
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */
void ISO9141_2_UART_init(void);
void ISO9141_2_UART_Transmit(uint8_t data);
uint8_t ISO9141_2_UART_Receive(void);
