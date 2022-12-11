/*
 * PersonaINFO.h
 *
 *  Created on: Dec 10, 2022
 *     	Author: owl_hor
 *      Storing personal information for displaying in Display
 */

#ifndef SRC_PERSONALINFO_PERSONAINFO_H_
#define SRC_PERSONALINFO_PERSONAINFO_H_

#include <stdio.h>
#include <strings.h>

#define PERSONA_LEN 12

//#define px_ID_search_datasss

/* fonts.h ex
typedef struct _tFont
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;

} sFONT;

extern sFONT Font24;
extern sFONT Font20;
extern sFONT Font16;
extern sFONT Font12;
extern sFONT Font8;

----in font_.c-------------------------
const uint8_t Font12_Table[] = {X,X,X,x,x,x,x,x,x,x,}

sFONT Font12 = {
  Font12_Table,
  7,  Width
  12,  Height
}

 * */
typedef struct _personna{
	uint8_t USID[5]; //// ID of RFID Card for that user
	char Name[20];
	char Surname[20];
	char welcom_txt[20];
	const uint16_t *pic;
	/*in sd card later
	 * TimeIN / TimeOUT / Counter
	 * */
}Personna;

extern Personna p1_owl;

#ifdef px_ID_search_datasss
extern Personna px_person[10];
#endif

#endif /* SRC_PERSONALINFO_PERSONAINFO_H_ */
