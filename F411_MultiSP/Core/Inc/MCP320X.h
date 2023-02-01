/*
 * MCP320X.h
 *
 *  Created on: 31 Jan 2566
 *      Author: owl_hor
 */

#ifndef INC_MCP320X_H_
#define INC_MCP320X_H_

#include "stm32f4xx_hal.h"

typedef union _MCP3202_SET{
	struct MC2
	{
		uint16_t bitread :12;
		uint16_t reserv :2; // LSB
		enum _REQFig{M_Diff_01,M_Diff_10,M_SE_CH0,M_SE_CH1} REQFig :2; // MSB
	}MCP3202_U;
	uint16_t U16;
}MCP3202_SET;


typedef union _MCP3208_SET{
	struct MC8
	{
		/* from the highest order byte to the lowest order byte and within each byte, from the LSB to the MSB.
		 * Bit 0 LSB
		 * Bit 0 MSB
		 * Bit 1 LSB
		 * Bit 1 MSB
		 * */
		//// Start bit[1] + SGL/DIFF + D2 D1 D0


		enum _CHSlct{M8_DF01 = 0b10000, M8_DF10, M8_DF23, M8_DF32, M8_DF45, M8_DF54, M8_DF67, M8_DF76,
			M8_CH0, M8_CH1, M8_CH2, M8_CH3, M8_CH4, M8_CH5, M8_CH6, M8_CH7} CHSlct:5;
		uint16_t reserv_start :5;
		uint16_t resr_null :2;
		uint16_t bitread :12;
		/* DFxy = Differential, x channel is IN+, y channel is IN-
		 * CHx  = single-ended at x channel
		 * */

	}MCP3208_U;
	//uint16_t U16[2];
	uint8_t U8[3];
}MCP3208_SET;

//typedef union _MCP3208_CMD{
//	struct MCC8
//	{
//		/* from the highest order byte to the lowest order byte and within each byte, from the LSB to the MSB.
//		 * Bit 0 LSB
//		 * Bit 0 MSB
//		 * Bit 1 LSB
//		 * Bit 1 MSB
//		 * */
//		//// Start bit[1] + SGL/DIFF + D2 D1 D0
//
//		/* DFxy = Differential, x channel is IN+, y channel is IN-
//		 * CHx  = single-ended at x channel
//		 * */
//		enum _CHSct{M8_DF01 = 0b10000, M8_DF10, M8_DF23, M8_DF32, M8_DF45, M8_DF54, M8_DF67, M8_DF76,
//			M8_CH0, M8_CH1, M8_CH2, M8_CH3, M8_CH4, M8_CH5, M8_CH6, M8_CH7} CHSct:5;
//		uint16_t reserv_start :5;
//		uint16_t resrv_null :6;
//
//
//	}MCP3208_U;
//	//uint16_t U16[2];
//	uint8_t U8[2];
//}MCP3208_CMD;

void MCP3202_READ(SPI_HandleTypeDef *hspi, uint16_t pTrX, uint16_t pRcX);
void MCP3208_READ(SPI_HandleTypeDef *hspi, uint16_t pTrX, uint16_t pRcX);

#endif /* INC_MCP320X_H_ */
