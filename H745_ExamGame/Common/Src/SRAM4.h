/*
 * SRAM4.h
 *
 *  Created on: Jan 10, 2023
 *      Author: User
 */

#ifndef SRC_SRAM4_H_
#define SRC_SRAM4_H_

#define SHARED_MEMORY_ADDR 0x38000000 // SRAM4 start addr(in datasheet)

//// anything write in main "Common" folder will auto sync to CM7 CM4 's "Common"
//5
//// make S4 ram mem access for dual core to realize same struct
//// write at .h here -> easy to edit for 2 cores
typedef struct
{
	uint8_t flag_blue_btn;
	//uint32_t DATA[500];

}SharedMem;

SharedMem *const SRAM4 = (SharedMem*)(SHARED_MEMORY_ADDR);

#endif /* SRC_SRAM4_H_ */
