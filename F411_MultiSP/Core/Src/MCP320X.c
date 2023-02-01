/*
 * MCP320X.c
 *
 *  Created on: 31 Jan 2566
 *      Author: owl_hor
 */

#include "MCP320X.h"

#define MCP3202_SPI_CS_Port GPIOA
#define MCP3202_SPI_CS_Pin  GPIO_PIN_8

#define MCP3208_SPI_CS_Port  GPIOA
#define MCP3208_SPI_CS_Pin   GPIO_PIN_9

/* Motorola 16 bit MSB
 * */

void MCP3202_READ(SPI_HandleTypeDef *hspi, uint16_t pTrX, uint16_t pRcX){

	//uint16_t datain = pTrX; //0b1100000000000000
	HAL_GPIO_WritePin(MCP3202_SPI_CS_Port,MCP3202_SPI_CS_Pin , GPIO_PIN_RESET);
	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);

	//HAL_SPI_TransmitReceive_IT(hspi, pTrX, pRcX, 1);
	//HAL_SPI_TransmitReceive(hspi, pTrX, pRcX, 1, 100);

	//// Dout = ( 4096 x Vin )/ VCC
	//// Dout x VCC / 4096 = Vin
	HAL_Delay(10);
	HAL_GPIO_WritePin(MCP3202_SPI_CS_Port, MCP3202_SPI_CS_Pin, GPIO_PIN_SET);

}

//void MCP3208_READ(SPI_HandleTypeDef *hspi, uint16_t pTrX, uint16_t pRcX){
//
//	//uint16_t datain = pTrX; //0b1100000000000000
//	HAL_GPIO_WritePin(MCP3208_SPI_CS_Port,MCP3208_SPI_CS_Pin , GPIO_PIN_RESET);
//	//HAL_SPI_Transmit_IT(&hspi2, &datain, 1);
//	//HAL_SPI_TransmitReceive(&hspi2, pTrX, &pRcX, 1, 100);
//
//	HAL_SPI_TransmitReceive_IT(hspi, pTrX, pRcX, 1);
//	//HAL_SPI_TransmitReceive(hspi, pTrX, pRcX, 1, 100);
//
//	//// Dout = ( 4096 x Vin )/ VCC
//	//// Dout x VCC / 4096 = Vin
//
//}


