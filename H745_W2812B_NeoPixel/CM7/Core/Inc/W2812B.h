/*
 * W2812B.h
 *
 *  Created on: Oct 6, 2022
 *      Author: User
 */

#ifndef INC_W2812B_H_
#define INC_W2812B_H_

#include "stm32h7xx_hal.h"

/////////////// ProtoTypo ///////////
typedef struct
{
	uint8_t R,G,B;
}W2812BStructure;
void W2812B_Init(SPI_HandleTypeDef *hspi);

void W2812B_UpdateData(W2812BStructure *input);
void HToRGB(int h,W2812BStructure *input);
void W28_Cf_R_G_B(int rr, int gg, int bb,W2812BStructure *input);
void HexToRGB_cat(uint32_t hx,W2812BStructure *input);

#endif /* INC_W2812B_H_ */
