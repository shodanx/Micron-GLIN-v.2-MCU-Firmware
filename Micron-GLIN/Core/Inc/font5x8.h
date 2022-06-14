/*
 * font5x8.h
 *
 *  Created on: Jun 14, 2022
 *      Author: User
 */

#ifndef INC_FONT5X8_H_
#define INC_FONT5X8_H_

/********************************** Таблица символов **********************************/

static char lcd_font_table[0x01E0] = {
  0x00, 0x00, 0x00, 0x00, 0x00, // 20 space
  0x00, 0x00, 0x5F, 0x00, 0x00, // 21 !
  0x00, 0x07, 0x00, 0x07, 0x00, // 22 "
  0x14, 0x7F, 0x14, 0x7F, 0x14, // 23 #
  0x24, 0x2A, 0x7F, 0x2A, 0x12, // 24 $
  0x23, 0x13, 0x08, 0x64, 0x62, // 25 %
  0x36, 0x49, 0x55, 0x22, 0x50, // 26 &
  0x44, 0x44, 0x5F, 0x44, 0x44, // 27 ' (заменен на символ +-)
  0x00, 0x1C, 0x22, 0x41, 0x00, // 28 (
  0x00, 0x41, 0x22, 0x1C, 0x00, // 29 )
  0x14, 0x08, 0x3E, 0x08, 0x14, // 2a *
  0x08, 0x08, 0x3E, 0x08, 0x08, // 2b +
  0x00, 0x50, 0x30, 0x00, 0x00, // 2c ,
  0x08, 0x08, 0x08, 0x08, 0x08, // 2d -
  0x00, 0x60, 0x60, 0x00, 0x00, // 2e .
  0x20, 0x10, 0x08, 0x04, 0x02, // 2f /
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 30 0
  0x00, 0x42, 0x7F, 0x40, 0x00, // 31 1
  0x42, 0x61, 0x51, 0x49, 0x46, // 32 2
  0x21, 0x41, 0x45, 0x4B, 0x31, // 33 3
  0x18, 0x14, 0x12, 0x7F, 0x10, // 34 4
  0x27, 0x45, 0x45, 0x45, 0x39, // 35 5
  0x3C, 0x4A, 0x49, 0x49, 0x30, // 36 6
  0x01, 0x71, 0x09, 0x05, 0x03, // 37 7
  0x36, 0x49, 0x49, 0x49, 0x36, // 38 8
  0x06, 0x49, 0x49, 0x29, 0x1E, // 39 9
  0x00, 0x36, 0x36, 0x00, 0x00, // 3a :
  0x00, 0x56, 0x36, 0x00, 0x00, // 3b ;
  0x08, 0x14, 0x22, 0x41, 0x00, // 3c <
  0x14, 0x14, 0x14, 0x14, 0x14, // 3d =
  0x00, 0x41, 0x22, 0x14, 0x08, // 3e >
  0x02, 0x01, 0x51, 0x09, 0x06, // 3f ?
  0x32, 0x49, 0x79, 0x41, 0x3E, // 40 @
  0x7E, 0x11, 0x11, 0x11, 0x7E, // 41 A
  0x7F, 0x49, 0x49, 0x49, 0x36, // 42 B
  0x3E, 0x41, 0x41, 0x41, 0x22, // 43 C
  0x7F, 0x41, 0x41, 0x22, 0x1C, // 44 D
  0x7F, 0x49, 0x49, 0x49, 0x41, // 45 E
  0x7F, 0x09, 0x09, 0x09, 0x01, // 46 F
  0x3E, 0x41, 0x49, 0x49, 0x7A, // 47 G
  0x7F, 0x08, 0x08, 0x08, 0x7F, // 48 H
  0x00, 0x41, 0x7F, 0x41, 0x00, // 49 I
  0x20, 0x40, 0x41, 0x3F, 0x01, // 4a J
  0x7F, 0x08, 0x14, 0x22, 0x41, // 4b K
  0x7F, 0x40, 0x40, 0x40, 0x40, // 4c L
  0x7F, 0x02, 0x0C, 0x02, 0x7F, // 4d M
  0x7F, 0x04, 0x08, 0x10, 0x7F, // 4e N
  0x3E, 0x41, 0x41, 0x41, 0x3E, // 4f O
  0x7F, 0x09, 0x09, 0x09, 0x06, // 50 P
  0x3E, 0x41, 0x51, 0x21, 0x5E, // 51 Q
  0x7F, 0x09, 0x19, 0x29, 0x46, // 52 R
  0x46, 0x49, 0x49, 0x49, 0x31, // 53 S
  0x01, 0x01, 0x7F, 0x01, 0x01, // 54 T
  0x3F, 0x40, 0x40, 0x40, 0x3F, // 55 U
  0x1F, 0x20, 0x40, 0x20, 0x1F, // 56 V
  0x3F, 0x40, 0x38, 0x40, 0x3F, // 57 W
  0x63, 0x14, 0x08, 0x14, 0x63, // 58 X
  0x07, 0x08, 0x70, 0x08, 0x07, // 59 Y
  0x61, 0x51, 0x49, 0x45, 0x43, // 5a Z
  0x00, 0x7F, 0x41, 0x41, 0x00, // 5b [
  0x02, 0x04, 0x08, 0x10, 0x20, // 5c Yen Currency Sign
  0x00, 0x41, 0x41, 0x7F, 0x00, // 5d ]
  0x04, 0x02, 0x01, 0x02, 0x04, // 5e ^
  0x40, 0x40, 0x40, 0x40, 0x40, // 5f _
  0x00, 0x01, 0x02, 0x04, 0x00, // 60 `
  0x20, 0x54, 0x54, 0x54, 0x78, // 61 a
  0x7F, 0x48, 0x44, 0x44, 0x38, // 62 b
  0x38, 0x44, 0x44, 0x44, 0x20, // 63 c
  0x38, 0x44, 0x44, 0x48, 0x7F, // 64 d
  0x38, 0x54, 0x54, 0x54, 0x18, // 65 e
  0x08, 0x7E, 0x09, 0x01, 0x02, // 66 f
  0x0C, 0x52, 0x52, 0x52, 0x3E, // 67 g
  0x7F, 0x08, 0x04, 0x04, 0x78, // 68 h
  0x00, 0x44, 0x7D, 0x40, 0x00, // 69 i
  0x20, 0x40, 0x44, 0x3D, 0x00, // 6a j
  0x7F, 0x10, 0x28, 0x44, 0x00, // 6b k
  0x00, 0x41, 0x7F, 0x40, 0x00, // 6c l
  0x7C, 0x04, 0x18, 0x04, 0x78, // 6d m
  0x7C, 0x08, 0x04, 0x04, 0x78, // 6e n
  0x38, 0x44, 0x44, 0x44, 0x38, // 6f o
  0x7C, 0x14, 0x14, 0x14, 0x08, // 70 p
  0x08, 0x14, 0x14, 0x18, 0x7C, // 71 q
  0x7C, 0x08, 0x04, 0x04, 0x08, // 72 r
  0x08, 0x54, 0x54, 0x54, 0x20, // 73 s
  0x04, 0x3F, 0x44, 0x40, 0x20, // 74 t
  0x3C, 0x40, 0x40, 0x20, 0x7C, // 75 u
  0x1C, 0x20, 0x40, 0x20, 0x1C, // 76 v
  0x3C, 0x40, 0x30, 0x40, 0x3C, // 77 w
  0x44, 0x28, 0x10, 0x28, 0x44, // 78 x
  0x0C, 0x50, 0x50, 0x50, 0x3C, // 79 y
  0x44, 0x64, 0x54, 0x4C, 0x44, // 7a z
  0x00, 0x08, 0x36, 0x41, 0x00, // 7b <
  0x00, 0x00, 0x7F, 0x00, 0x00, // 7c |
  0x00, 0x41, 0x36, 0x08, 0x00, // 7d >
  0x10, 0x08, 0x08, 0x10, 0x08, // 7e Right Arrow ->
  0x78, 0x46, 0x41, 0x46, 0x78  // 7f Left Arrow <-
};

static char lcd_font_table_rus[0x0140] = {
  0x7E, 0x11, 0x11, 0x11, 0x7E, // C0 А
  0x7F, 0x49, 0x49, 0x49, 0x31, // C1 Б
  0x7F, 0x49, 0x49, 0x49, 0x36, // C2 В
  0x7F, 0x01, 0x01, 0x01, 0x03, // C3 Г
  0x60, 0x3E, 0x21, 0x21, 0x7F, // C4 Д
  0x7F, 0x49, 0x49, 0x49, 0x41, // C5 Е
  0x77, 0x08, 0x7F, 0x08, 0x77, // C6 Ж
  0x22, 0x41, 0x49, 0x49, 0x36, // C7 З
  0x7F, 0x10, 0x08, 0x04, 0x7F, // C8 И
  0x7F, 0x10, 0x09, 0x04, 0x7F, // C9 И
  0x7F, 0x08, 0x14, 0x22, 0x41, // CA К
  0x40, 0x3E, 0x01, 0x01, 0x7F, // CB Л
  0x7F, 0x02, 0x0C, 0x02, 0x7F, // CC М
  0x7F, 0x08, 0x08, 0x08, 0x7F, // CD Н
  0x3E, 0x41, 0x41, 0x41, 0x3E, // CE О
  0x7F, 0x01, 0x01, 0x01, 0x7F, // CF П
  0x7F, 0x09, 0x09, 0x09, 0x06, // D0 Р
  0x3E, 0x41, 0x41, 0x41, 0x22, // D1 С
  0x01, 0x01, 0x7F, 0x01, 0x01, // D2 Т
  0x27, 0x48, 0x48, 0x48, 0x3F, // D3 У
  0x1E, 0x21, 0x7F, 0x21, 0x1E, // D4 Ф
  0x63, 0x14, 0x08, 0x14, 0x63, // D5 Х
  0x3F, 0x20, 0x20, 0x3F, 0x60, // D6 Ц
  0x07, 0x08, 0x08, 0x08, 0x7F, // D7 Ч
  0x7F, 0x40, 0x7F, 0x40, 0x7F, // D8 Ш
  0x3F, 0x20, 0x3F, 0x20, 0x7F, // D9 Щ
  0x01, 0x7F, 0x48, 0x48, 0x30, // DA Ъ
  0x7F, 0x48, 0x30, 0x00, 0x7F, // DB Ы
  0x00, 0x7F, 0x48, 0x48, 0x30, // DC Ь
  0x22, 0x41, 0x49, 0x49, 0x3E, // DD Э
  0x7F, 0x08, 0x3E, 0x41, 0x3E, // DE Ю
  0x46, 0x29, 0x19, 0x09, 0x7F, // DF Я
  0x20, 0x54, 0x54, 0x54, 0x78, // E0 а
  0x3C, 0x4A, 0x4A, 0x4A, 0x30, // E1 б
  0x7C, 0x54, 0x54, 0x28, 0x00, // E2 в
  0x7C, 0x04, 0x04, 0x04, 0x04, // E3 г
  0x60, 0x38, 0x24, 0x24, 0x7C, // E4 д
  0x38, 0x54, 0x54, 0x54, 0x18, // E5 е
  0x6C, 0x10, 0x7C, 0x10, 0x6C, // E6 ж
  0x00, 0x44, 0x54, 0x54, 0x28, // E7 з
  0x7C, 0x20, 0x10, 0x08, 0x7C, // E8 и
  0x7C, 0x21, 0x12, 0x09, 0x7C, // E9 й
  0x7C, 0x10, 0x28, 0x44, 0x00, // EA к
  0x40, 0x38, 0x04, 0x04, 0x7C, // EB л
  0x7C, 0x08, 0x10, 0x08, 0x7C, // EC м
  0x7C, 0x10, 0x10, 0x10, 0x7C, // ED н
  0x38, 0x44, 0x44, 0x44, 0x38, // EE о
  0x7C, 0x04, 0x04, 0x04, 0x7C, // EF п
  0x7C, 0x14, 0x14, 0x14, 0x08, // F0 р
  0x38, 0x44, 0x44, 0x44, 0x00, // F1 с
  0x04, 0x04, 0x7C, 0x04, 0x04, // F2 т
  0x0C, 0x50, 0x50, 0x50, 0x3C, // F3 у
  0x08, 0x14, 0x7C, 0x14, 0x08, // F4 ф
  0x44, 0x28, 0x10, 0x28, 0x44, // F5 х
  0x3C, 0x20, 0x20, 0x3C, 0x60, // F6 ц
  0x0C, 0x10, 0x10, 0x10, 0x7C, // F7 ч
  0x7C, 0x40, 0x7C, 0x40, 0x7C, // F8 ш
  0x3C, 0x20, 0x3C, 0x20, 0x7C, // F9 щ
  0x04, 0x7C, 0x50, 0x50, 0x20, // FA ъ
  0x7C, 0x50, 0x20, 0x00, 0x7C, // FB ы
  0x00, 0x7C, 0x50, 0x50, 0x20, // FC ь
  0x28, 0x44, 0x54, 0x54, 0x38, // FD э
  0x7C, 0x10, 0x38, 0x44, 0x38, // FE ю
  0x48, 0x54, 0x34, 0x14, 0x7C  // FF я
};

/**************************************************************************************/



#endif /* INC_FONT5X8_H_ */
