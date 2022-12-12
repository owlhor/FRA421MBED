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
//#include <strings.h>

#define px_ID_search_datasss
#define FOR_compile_test

typedef struct _personna{
	const uint8_t USID[5]; //// ID of RFID Card for that user
	const char Name[20];
	const char Surname[20];
	const char welcom_txt[50];
	const uint16_t *pic;  //// RGB565 Pic format
	const uint16_t picXs; //// bitmap pic size x axis
	const uint16_t picYs; //// bitmap pic size y axis
	/*in sd card later
	 * TimeIN / TimeOUT / Counter
	 * */
}Personna;

//// single test
extern Personna p1_owl;


//// test ---------/-----------/---------------

//typedef struct _mystructxt {
//
//  //const char * const l_name; //
//  uint8_t x_m;
//  uint8_t y_m;
//  //uint16_t period[4];
//} myStructxt;

/* fonts.h ex --------------------------------------------------------------
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
------------------------------------------------------------------------------
 * */


#endif /* SRC_PERSONALINFO_PERSONAINFO_H_ */
