/*
 * persona_2.h
 *
 *  Created on: 12 Dec 2565
 *      Author:owl_hor
 */

#ifndef SRC_PERSONALINFO_PERSONA_2_H_
#define SRC_PERSONALINFO_PERSONA_2_H_

#include "PersonaINFO.h"
#include "persona_3.h" //// pic

#define PERSONA_LEN 5

const Personna px_person_0 = {
		{0x0E, 0x39, 0x62, 0x5B, 0x00},
		"Aj PI Pitiwut",
		"Teerakittikul",
		"--- Electronics Lecturer ---",
		px0_PIC,
		128, 128
};

const Personna px_person_1 = {
		{0x90, 0x5B, 0xD9, 0xA4, 0x00},
		"P PUN Puttinart",
		"Archeewawanich",
		"----- Super TA -----",
		px1_PIC,
		126, 127
};

const Personna px_person_2 = {
		{0xA0, 0x1D, 0xE2, 0xA4, 0x00},
		"Wipop",
		"Panyatipsakul",
		"---  OF Chairman  ---",
		px2_PIC,
		128, 128
};

//const Personna px_person_3 = {
//		{0x90, 0x5B, 0xD9, 0xA4, 0x00},
//		"P PUN Puttinart",
//		"Archeewawanich",
//		" ----- Super TA -----",
//		px1_PIC,
//		126, 127
//};

Personna pxs_persons[] = {
		px_person_0,
		px_person_1,
		px_person_2

};


#endif /* SRC_PERSONALINFO_PERSONA_2_H_ */
