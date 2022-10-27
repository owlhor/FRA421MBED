/*
 * fonts.c
 *
 *  Created on: Oct 27, 2022
 *      Author: owl_hor
 *      Ref.  : ScarsFun
 */

#ifndef INC_FONTS_H_
#define INC_FONTS_H_

#include <stdint.h>

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint16_t *data;
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;


#endif /* INC_FONTS_H_ */
