/* Orientation clockwise
   - 0: 240x320 portrait 0'
   - 1: 320x240 landscape 90'
   - 2: 240x320 portrait 180'
   - 3: 320x240 landscape 270'
*/
#define  ILI9486_ORIENTATION      1

/* Color mode
   - 0: RGB565 (R:bit15..11, G:bit10..5, B:bit4..0)
   - 1: BRG565 (B:bit15..11, G:bit10..5, R:bit4..0)
*/
#define  ILI9486_COLORMODE        0

/* Analog touchscreen
   - 0: touchscreen disabled
   - 1: touchscreen enabled
*/
#define  ILI9486_TOUCH            0

/* Touchscreen calibration data for 4 orientations */
#define  TS_CINDEX_0        {3385020, 333702, -667424, 1243070964, -458484, -13002, 1806391572}
#define  TS_CINDEX_1        {3385020, -458484, -13002, 1806391572, -333702, 667424, -163249584}
#define  TS_CINDEX_2        {3385020, -333702, 667424, -163249584, 458484, 13002, -184966992}
#define  TS_CINDEX_3        {3385020, 458484, 13002, -184966992, 333702, -667424, 1243070964}

/* For multi-threaded or interrupt use, Lcd and Touchscreen simultaneous use can cause confusion (since it uses common I/O resources)
   If enabled, the Lcd functions wait until the touchscreen functions are run. The touchscreen query is not executed when Lcd is busy.
   - 0: multi-threaded protection disabled
   - 1: multi-threaded protection enabled
*/
#define  ILI9486_MULTITASK_MUTEX   0

//-----------------------------------------------------------------------------
// ILI9486 physic resolution (in 0 orientation)
#define  ILI9486_LCD_PIXEL_WIDTH  320
#define  ILI9486_LCD_PIXEL_HEIGHT 480

/*****************************************************************************/
// Color Definitions
/*****************************************************************************/

#define cl_BLACK       0x0000
#define cl_NAVY        0x000F
#define cl_DARKGREEN   0x03E0
#define cl_DARKCYAN    0x03EF
#define cl_MAROON      0x7800
#define cl_PURPLE      0x780F
#define cl_OLIVE       0x7BE0
#define cl_LIGHTGREY   0xC618
#define cl_GRAY        0x5AEB
#define cl_DARKGREY    0x7BEF
#define cl_BLUE        0x001F
#define cl_GREEN       0x07E0
#define cl_CYAN        0x07FF
#define cl_RED         0xF800
#define cl_MAGENTA     0xF81F
#define cl_YELLOW      0xFFE0
#define cl_WHITE       0xFFFF
#define cl_ORANGE      0xFD20
#define cl_GREENYELLOW 0xAFE5
#define cl_PINK        0xF81F

/*****************************************************************************/
// Lcd Prototypo
/*****************************************************************************/

#include "Fonts/fonts.h"

void     ili9486_Init(void);
uint16_t ili9486_ReadID(void);
void     ili9486_DisplayOn(void);
void     ili9486_DisplayOff(void);
void     ili9486_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     ili9486_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);
uint16_t ili9486_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void     ili9486_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     ili9486_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9486_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     ili9486_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t RGBCode);
uint16_t ili9486_GetLcdPixelWidth(void);
uint16_t ili9486_GetLcdPixelHeight(void);
void     ili9486_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void     ili9486_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9486_ReadRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint16_t *pData);
void     ili9486_Scroll(int16_t Scroll, uint16_t TopFix, uint16_t BottonFix);
void 	 ili9486_WriteChar(uint16_t Xpo, uint16_t Ypo, char *chr,sFONT fonto, uint16_t RGB_Coder, uint16_t RGB_bg);
void 	 ili9486_WriteCharNoBG(uint16_t Xpo, uint16_t Ypo, char *chr,sFONT fonto, uint16_t RGB_Coder);
void 	 ili9486_WriteString(uint16_t Xpo, uint16_t Ypo,const char* strr,sFONT fonto, uint16_t RGB_Coder, uint16_t RGB_bg);
void     ili9486_WriteStringNoBG(uint16_t Xpo, uint16_t Ypo,const char* strr,sFONT fonto, uint16_t RGB_Coder);
/* Touchscreen */
void     ili9486_ts_Init(uint16_t DeviceAddr);
uint8_t  ili9486_ts_DetectTouch(uint16_t DeviceAddr);
void     ili9486_ts_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y);
