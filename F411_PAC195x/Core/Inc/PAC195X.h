/*
 * PAC195X.h
 *
 *  Created on: Oct 7, 2022
 *      Author: OWL_HOR
 *      Library for PAC195X, MICROCHIP I2C
 *      Power Monitor Management
 *      Test on NucleoF411RE Board
 */

#ifndef INC_PAC195X_H_
#define INC_PAC195X_H_

#include "stm32f4xx_hal.h"

#define PAC_Regis_PDID 0xFD

//enum _NrmAcLat{NRM, ACT, LAT}NrmAcLat;
//typedef union __PAC_NrmACTLAT{
//	enum _NrmAcLat{NRM, ACT, LAT}NrmAcLat;
//	uint8_t D8;
//}_PAC_NrmACTLAT;

typedef union _PAC_CTRL_Strc{
	struct PACCTRL{
		//LSB  I2C Frame#2
		enum _SLOW_ALERT1{ ALERT1, GPIO1_IN, GPIO1_OUT, SLOW1
		} SLOW_ALERT1 :2;
		enum _GPIO_ALERT2{ ALERT2, GPIO2_IN, GPIO2_OUT, SLOW2
		} GPIO_ALERT2 :2;
		enum _SAMPLE_MODE{S_1024_AA, S_256AA, S_64_AA, S_8_AA,
						S_1024, S_256, S_64, S_8,
						Single_shot, Single_Shot8X, Fast_M, Burst_M,
						Rsv1, Rsv2, Rsv3, Sleep
		} SAMPLE_MODE :4;
		// MSB  I2C Frame#2

		// LSB  I2C Frame#1
		uint8_t Reserv :4;
		uint8_t CHANNEL_N_OFF :4;
		// MSB  I2C Frame#1
	}PACCTRL;
	uint16_t D16;
	uint8_t D8[2];
}PAC_CTRL_Strc;

typedef union _PAC_PD_ID_Strc{
	struct PACPDID{
		// LSB  I2C Frame#1
		uint8_t Produc_ID;
		uint8_t Manu_54h_ID;
		uint8_t Rev_ID;
		uint8_t reserv;
		// MSB  I2C Frame#1
	}PACPDID;
	uint8_t D8[4];
}PAC_PD_ID_Strc;

/*
 * ACC_COUNT
 * VACC[1,4]
 * VBUS[1,4]
 * VSENSE[1,4]
 * VBUS[1,4]_AVG
 * VSENSE[1,4]_AVG
 * VPOWER[1,4]
 * OC[1,4] Limit
 * UC[1,4] Limit
 * OP[1,4] Limit
 * OV[1,4] Limit
 * UV[1,4] Limit
 *
 * OC_ UC_ OP_ UP_ OV_ UV_ LimitNsamples
 *
 * SMBUS_SETTINGS
 * SLOW
 * ALERT_STATUS
 * ALERT_ENABLE
 * SLOW_ALERT1
 * GPIO_ALERT2
 * ACC_FULLNESS
 *
 * NEG_PWR_FSR _ACT _LAT
 * ACCUM_CONFIG  _ACT _LAT
 * ----Completed---- CTRL _ACT _LAT
 * */

float PAC_Cal_BusVolt(uint16_t rawbus);
float PAC_Cal_SnsCurrent(uint16_t vsens);
void PAC_Read_ID(I2C_HandleTypeDef *hi2c, uint8_t DevADDR);
void PAC_Read_CTRL(I2C_HandleTypeDef *hi2c, uint8_t DevADDR,uint8_t LATt);

#endif /* INC_PAC195X_H_ */
