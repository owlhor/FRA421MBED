/*
 * SRAM4.h
 *
 *  Created on: Oct 27, 2022
 *      Author: User
 */

#ifndef SRC_SRAM4_H_
#define SRC_SRAM4_H_
//#define TASK2_NO_HSEM //// HSEM => Semaphore
//#define TASK2_WITH_HSEM

#define SHARED_MEMORY_ADDR 0x38000000 // SRAM4 start addr(in datasheet)

//// anything write in main "Common" folder will auto sync to CM7 CM4 's "Common"
//5
//// make S4 ram mem access for dual core to realize same struct
//// write at .h here -> easy to edit for 2 cores
typedef struct
{
	uint32_t state1;
	uint32_t DATA[500];

}SharedMem;

SharedMem *const SRAM4 = (SharedMem*)(SHARED_MEMORY_ADDR);

//// sample with no struct
//uint32_t *sta1 = (uint32_t*)(0x38000000);
//uint8_t *sta2 = (uint32_t*)(0x38000004);
//uint32_t *sta3 = (uint32_t*)(0x38000005);


#endif /* SRC_SRAM4_H_ */
