/*
 * DS_RTC.h
 *
 *  Created on: Nov 18, 2022
 *      Author: owl_hor
 */

#ifndef INC_DS_RTC_H_
#define INC_DS_RTC_H_

#include "stm32h7xx_hal.h"

// 1101 000 (R/W) External RTC ADDRESS
#define DS_ADDR 0b11010000

#define ExRTC_IS_DS3231
#ifdef ExRTC_IS_DS3231

typedef union _DS3231_RG_Strc{
	struct DS3231RG{
		//// 00h
		uint8_t sec: 8;
		//// 01h
		uint8_t min: 8;
		//// 02h
		uint8_t hour: 7;
		enum _HourMode1224{H24M, H12M
		} HourMode: 1;
		//// 03h
		enum _wkday{NA, Sun, Mon, Tue, Wed, Thur, Fri, Sat
		} wkday: 3;
		uint16_t reserv1: 5;
		//// 04h
		uint16_t date: 8;
		//// 05h
		uint16_t month :5;
		uint16_t reserv2 :2;
		uint16_t century :1;
		/// 06h
		uint16_t year: 8;
		//// 07h
		uint16_t sec_A1: 7;
		uint16_t A1M1: 1;
		//// 08h
		uint16_t min_A1: 7;
		uint16_t A1M2: 1;
		//// 09h
		uint16_t hour_A1: 6;
		enum _HourMode1224A1{H24M1, H12M1
		} HourMode_A1: 1;
		uint16_t A1M3: 1;
		//// 0Ah
		uint16_t day_A1: 6;
		enum _DYDTA1{DT1, DY1
		} DYDT_A1: 1;
		uint16_t A1M4: 1;
		//// 0Bh
		uint16_t min_A2: 7;
		uint16_t A2M2: 1;
		//// 0Ch
		uint16_t hour_A2: 6;
		enum _HourMode1224A2{H24M2, H12M2
		} HourMode_A2: 1;
		uint16_t A2M3: 1;
		//// 0Dh
		uint16_t day_A2: 6;
		enum _DYDTA2{DT2, DY2
		} DYDT_A2: 1;
		uint8_t A2M4: 1;
		//// 0Eh
		uint16_t C_A1IE: 1;
		uint16_t C_A2IE: 1;
		uint16_t C_INTCN: 1;
		uint16_t C_RS1: 1;
		uint16_t C_RS2: 1;
		uint16_t C_CONV: 1;
		uint16_t C_BBSQW: 1;
		uint16_t C_EOSC: 1;
		//// 0Fh
		uint16_t C_A1F: 1;
		uint16_t C_A2F: 1;
		uint16_t C_BSY: 1;
		uint16_t C_EN32KHZ: 1;
		uint16_t reserv3: 3;
		uint16_t C_OSF: 1;
		//// 11h
		uint16_t AgingOFFSet: 8;
		//// 12h
		uint16_t Temp: 10;
		uint16_t reserv4: 6;

	}DS3231RG;
	uint8_t D8[19];
}DS3231_RG_Strc;

void DS3231_Read(I2C_HandleTypeDef *hi2c);
void EXIN_RTC_SYNC(I2C_HandleTypeDef *hi2c, RTC_HandleTypeDef *hrtc);

#endif


#endif /* INC_DS_RTC_H_ */
