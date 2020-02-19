/*
 * ISO9141-2_low_level.h
 *
 *  Created on: Feb 17, 2020
 *      Author: kaktus
 */

#ifndef	ISO9141-29141_2_H
#define	ISO9141-29141_2_H

int8_t 		ISO9141_2_Init(void);
void 		ISO9141_2_WriteByte(uint8_t byte);
uint8_t 	ISO9141_2_ReadByte(void);
uint8_t		ISO9141_2_WriteData(uint8_t *data, uint8_t len);
uint8_t		ISO9141_2_ReadData(uint8_t *data, uint8_t len);
uint8_t 	ISO9141_2_Checksum(uint8_t *data, uint8_t len);

#endif /* ISO9141-29141_2_H */
