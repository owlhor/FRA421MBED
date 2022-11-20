/*
 * DS_RTC.c
 *
 *  Created on: Nov 18, 2022
 *      Author: owl_hor
 *
 *  Use Raspberry pi & import datetime to write timevalue into DS3231
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


//// Sync timevalue when boot(in case Vbat is not powered)
void EXIN_RTC_SYNC(I2C_HandleTypeDef *hi2c, RTC_HandleTypeDef *hrtc){
	DS3231_Read(hi2c);

	RTC_TimeTypeDef sTime={0};
	sTime.Hours = ERTC_lg.DS3231RG.hour; //
	sTime.Minutes = ERTC_lg.DS3231RG.min;
	sTime.Seconds = ERTC_lg.DS3231RG.sec;

	HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BCD);

	RTC_DateTypeDef sDate ={0};
	sDate.Date = ERTC_lg.DS3231RG.date;
	sDate.Month = ERTC_lg.DS3231RG.month;
	sDate.WeekDay = ERTC_lg.DS3231RG.wkday;
	sDate.Year = ERTC_lg.DS3231RG.year;

	HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BCD);
}



#endif
