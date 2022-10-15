/*
 * BMP_UART_Decode.h
 *
 *  Created on: Oct 14, 2022
 *      Author: owl_hor
 *      Ref.  : AlphaP
 */

#ifndef INC_BMP_UART_DECODE_H_
#define INC_BMP_UART_DECODE_H_

#include "stm32h7xx_hal.h"

void BMPDecoder(uint8_t dataIn, uint8_t *array);

#endif /* INC_BMP_UART_DECODE_H_ */
