/*
 * UART_LCD.h
 *
 *  Created on: Oct 14, 2022
 *      Author: owl_hor
 *      Ref.  : AlphaP
 */

#ifndef INC_UART_LCD_H_
#define INC_UART_LCD_H_

typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;


void UARTInit(UARTStucrture *uart);

void UARTResetStart(UARTStucrture *uart);

uint32_t UARTGetRxHead(UARTStucrture *uart);

int16_t UARTReadChar(UARTStucrture *uart);

void UARTTxDumpBuffer(UARTStucrture *uart);

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);




#endif /* INC_UART_LCD_H_ */
