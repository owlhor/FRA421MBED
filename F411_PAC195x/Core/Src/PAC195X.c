/*
 * PAC195X.c
 *
 *  Created on: Oct 7, 2022
 *      Author: OWL_HOR
 *      Library for PAC195X, MICROCHIP I2C
 *      Power Monitor Management
 *      Test on NucleoF411RE Board
 */

#include "PAC195X.h"

/// Equation from datasheet--------------------------------
#define  R_sense 5 // Ohm
// -- FSCurrent = 100mV / R_sense
// -- SnsCr_denom = (0.1 / R_sense) / 65536;
//float SnsCr_denom = 0.1 / (R_sense * 65536);
/// Equation from datasheet--------------------------------


float PAC_Cal_BusVolt(uint16_t rawbus){
	//float Denom = 0.000488281; // Vsource = 32 x Vbus/(2^16)
	return 0.000488281 * rawbus;
}

float PAC_Cal_SnsCurrent(uint16_t vsens){
	return vsens * 0.1 / (R_sense * 65536);
}


PAC_CTRL_Strc PAC_CTRL_read;
PAC_CTRL_Strc PAC_CTRL_ACT_read;
PAC_CTRL_Strc PAC_CTRL_LAT_read;
void PAC_Read_CTRL(I2C_HandleTypeDef *hi2c, uint8_t DevADDR,uint8_t LATt){ //

	/* Check by use these in live expression
	 * "PAC_CTRL_read" "PAC_CTRL_ACT_read" "PAC_CTRL_LAT_read"
	 * ex. PAC_Read_CTRL(&hi2c1, ADDR_PAC, 0);
	 * */

	uint8_t MemAddr;
	switch(LATt){
	case 0: // Normal
		MemAddr = 0x01; break;
	case 1: // ACT
		MemAddr = 0x21; break;
	case 2: // LAT
		MemAddr = 0x23; break;
	}

	if(hi2c->State == HAL_I2C_STATE_READY){
		HAL_I2C_Mem_Read(hi2c, DevADDR, MemAddr, I2C_MEMADD_SIZE_8BIT,
				&PAC_CTRL_read.D8[0], 2, 100);
	}
}

PAC_PD_ID_Strc PAC_ID_Chk;
void PAC_Read_ID(I2C_HandleTypeDef *hi2c, uint8_t DevADDR){ //

	/* Check by use "PAC_ID_Chk" in live expression
	 * ex. PAC_Read_ID(&hi2c1, ADDR_PAC);
	 * */
	if(hi2c->State == HAL_I2C_STATE_READY){
		HAL_I2C_Mem_Read(hi2c, DevADDR, PAC_Regis_PDID, I2C_MEMADD_SIZE_8BIT,
				&PAC_ID_Chk.D8[0], 3, 100);
	}
}


//void PAC_Read_8(uint8_t memadr,uint8_t *RawRd,uint8_t num){
//	if(hi2c1.State == HAL_I2C_STATE_READY){
//		HAL_I2C_Mem_Read(&hi2c1, ADDR_PAC, memadr, I2C_MEMADD_SIZE_8BIT,
//				  			RawRd,
//							num, 100);
//		//flag_pac = 0;
//		}
//}

