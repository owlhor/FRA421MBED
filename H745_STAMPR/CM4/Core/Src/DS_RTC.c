/*
 * DS_RTC.c
 *
 *  Created on: Nov 18, 2022
 *      Author: owl_hor
 */


#include "DS_RTC.h"

#ifdef ExRTC_IS_DS3231

DS3231_RG_Strc ERTC_lg;

void DS3231_Read(I2C_HandleTypeDef *hi2c){
	/* Check by use "ERTC_lg" in live expression
	 * ex. S3231_Read(&hi2c2);
	 * */
	if(hi2c->State == HAL_I2C_STATE_READY){
		HAL_I2C_Mem_Read(hi2c, DS_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT,
				&ERTC_lg.D8[0], 19, 100);
	}

}

#endif
