/*
 * SRAM4.h
 *
 *  Created on: Nov 11, 2022
 *      Author: owl_hor
 */

#ifndef SRC_SRAM4_H_
#define SRC_SRAM4_H_

#define SHARED_MEMORY_ADDR 0x38000000 // SRAM4 start addr(in datasheet)

//// anything write in main "Common" folder will auto sync to CM7 CM4 's "Common"
//// make S4 ram mem access for dual core to realize same struct
//// write at .h here -> easy to edit for 2 cores

/* HSEM 1 -> RTC
 * HSEM 2 -> UID
 * */

typedef struct
{
	uint32_t realtime_val;
	RTC_TimeTypeDef NowTimes;
	RTC_DateTypeDef NowDates;
	uint8_t flag_UID;
	uint8_t UUID[10];
	//uint32_t DATA[500];

}SharedMem;

SharedMem *const SRAM4 = (SharedMem*)(SHARED_MEMORY_ADDR);

#endif /* SRC_SRAM4_H_ */
