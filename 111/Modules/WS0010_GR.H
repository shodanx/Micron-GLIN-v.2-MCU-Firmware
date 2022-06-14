/*************************************************************************
 *    File name   : WS0010_GR.H
 *    Description : WS0010 graph driver definition
 *
 *    History :
 *    1. Date        : 12.07.2012
 *       Author      : S. Budakow
 *       Description : Create
 **************************************************************************/
#include "WS0010_inc.h"

#ifndef  __WS0010_GR_H
#define  __WS0010_GR_H

// This structure describes a single character's display information
typedef struct
{
	const Int8U widthBits;					// width, in bits (or pixels), of the character
	const Int16U offset;					// offset of the character's bitmap, in bytes, into the the FONT_INFO's data array

} FONT_CHAR_INFO;	

// Describes a single font
typedef struct
{
	const Int8U 			heightPages;	// height, in pages (8 pixels), of the font's characters
	const Int8U 			startChar;		// the first character in the font (e.g. in charInfo and data)
	const Int8U 			endChar;		// the last character in the font
	const Int8U			spacePixels;	// number of pixels that a space character takes up
	const FONT_CHAR_INFO* 	charInfo;		// pointer to array of char information
	const Int8U* 			data;			// pointer to generated array of character visual representation
		
} FONT_INFO;
	
//  3 880 bytes of readonly  code memory 
//  3 360 bytes of readonly  data memory 
//  6 916 bytes of readwrite data memory 
	

HD44780_ERROR_CODE_DEF WS0010_Command (Int8U Command);
HD44780_ERROR_CODE_DEF WS0010_Data (Int8U Data);
HD44780_ERROR_CODE_DEF WS0010_String_8 (Int16U PosX, Int16U PosY, 
                                    const HD44780_STRING_DEF * DataStr,
                                    const FONT_INFO* Font);
HD44780_ERROR_CODE_DEF WS0010_String_16 (Int16U PosX, Int16U PosY, 
                                    const HD44780_STRING_DEF * DataStr,
                                    const FONT_INFO* Font);
#ifdef WG0010_Graph_Mode
HD44780_ERROR_CODE_DEF HD44780_StrShow(HD44780_XY_DEF X, HD44780_XY_DEF Y, const HD44780_STRING_DEF * DataStr);
HD44780_ERROR_CODE_DEF WS0010_Sw2GR (void);
#endif
//функции из drv_hd44780.c
Int8U HD44780_GetDisplayPosition (void);
void HD44780WrComm (Int8U Command);
void HD44780WrData (Int8U Data);



#endif //ifndef  __WS0010_GR_H
