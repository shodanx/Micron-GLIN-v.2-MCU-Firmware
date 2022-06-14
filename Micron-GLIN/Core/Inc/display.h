/*
 * display.h
 *
 *  Created on: Jun 13, 2022
 *      Author: User
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "main.h"

#define PIXEL_OFF	0       // режимы отображения пикселя - используются в графических функциях
#define PIXEL_ON	1
#define PIXEL_XOR	2

#define LCD_X_RES               100      // разрешение экрана
#define LCD_Y_RES               16
#define LCD_CACHSIZE          LCD_X_RES*LCD_Y_RES>>3

#define Cntr_X_RES              100      // разрешение контроллера
#define Cntr_Y_RES              16
#define Cntr_buf_size           Cntr_X_RES*Cntr_Y_RE>>3

#define lcd_CMD 1
#define lcd_DATA 2

extern char lcd_buf[];


//***************************************************
//****************Прототипы функций******************
//***************************************************
void init_lcd_port(void);       // Инициализация порта LCD дисплея
void LcdInit(void);
void LcdClear(void);            //Clears the display
void LcdPixel(unsigned char x, unsigned char y, unsigned char mode);    //Displays a pixel at given absolute (x, y) location, mode -> Off, On or Xor
void LcdStringBold(unsigned char x, unsigned char y);   //Displays a string at current cursor location


void LcdUpdate(void);           //Copies the LCD cache into the device RAM
void LcdClear(void);            //Clears the display
void LcdPixel(unsigned char x, unsigned char y, unsigned char mode);    //Displays a pixel at given absolute (x, y) location, mode -> Off, On or Xor
void LcdLine(int x1, int y1, int x2, int y2, unsigned char mode);       //Draws a line between two points on the display
void LcdGotoXYFont(unsigned char x, unsigned char y);   //Sets cursor location to xy location. Range: 1,1 .. 14,6
void clean_lcd_buf(void);       //очистка текстового буфера
void LcdChr(int ch);            //Displays a character at current cursor location and increment cursor location
void LcdString(unsigned char x, unsigned char y);       //Displays a string at current cursor location
void LcdChrBold(int ch);        //Печатает символ на текущем месте, большой и жирный)
void LcdStringBold(unsigned char x, unsigned char y);   //Печатает большую и жирную строку
void LcdChrBig(int ch);         //Печатает символ на текущем месте, большой
void LcdStringBig(unsigned char x, unsigned char y);    //Печатает большую строку
//***************************************************
// UPDATE ##1
void LcdBar(int x1, int y1, int x2, int y2, unsigned char persent);     // рисует прогресс-бар и заполняет его на "процент"
void LcdBarLine(unsigned char line, unsigned char persent);     // рисуем прошресс-бар в указанной строке
void LcdStringInv(unsigned char x, unsigned char y);    // печатает строку в инверсном шрифте (удобно для настроек)

void LcdClear_massive(void);

#define HD44780_SET_CGRAM_ADD         0x40    /* Set AC point to CGRAM */
#define HD44780_SET_DDRAM_ADD         0x80    /* Set AC point to DDRAM */


#define lcd44780_RS_0      HAL_GPIO_WritePin(Display_RS_GPIO_Port, Display_RS_Pin, GPIO_PIN_SET);
#define lcd44780_RS_1      HAL_GPIO_WritePin(Display_RS_GPIO_Port, Display_RS_Pin, GPIO_PIN_RESET);


#define lcd44780_E_1       HAL_GPIO_WritePin(Display_EN_GPIO_Port, Display_EN_Pin, GPIO_PIN_SET);
#define lcd44780_E_0       HAL_GPIO_WritePin(Display_EN_GPIO_Port, Display_EN_Pin, GPIO_PIN_RESET);

#define lcd44780_RW_0      HAL_GPIO_WritePin(Display_RW_GPIO_Port, Display_RW_Pin, GPIO_PIN_RESET);
#define lcd44780_RW_1      HAL_GPIO_WritePin(Display_RW_GPIO_Port, Display_RW_Pin, GPIO_PIN_SET);


void pulse_e();
void send_nibble(unsigned char data);
void send_data(unsigned char data);
void init_LCD();
int check_busy_flag(void);
void init_LCD_pins();
void init_BTN_pins();
void lcd_write_byte(unsigned char addr, unsigned char data);

#endif /* INC_DISPLAY_H_ */
