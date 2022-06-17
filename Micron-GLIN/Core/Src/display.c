/*
 * display.c
 *
 *  Created on: Jun 13, 2022
 *      Author: User
 */

#include "display.h"
#include "font5x8.h"

char lcd_buf[20];               // текстовый буфер для вывода на LCD
unsigned char LcdCache[LCD_CACHSIZE];   // Фреймбуфер
unsigned int LcdCacheIdx = 0;   // Текущий адрес во фреймбуфере


void LcdSend(uint8_t data, uint8_t cmd) //Sends data to display controller
{
  uint8_t sdata = 0;

  lcd44780_RW_0;

  sdata = data;
  if(cmd == lcd_CMD)
  {
	  lcd44780_RS_0;
  }                             //идентефикатор кода
  else
  {
	  lcd44780_RS_1;
  }                             //идентефикатор данных
  send_data(sdata);

  if(cmd == lcd_CMD)while(check_busy_flag());
}

void LcdUpdate(void)            //Copies the LCD cache into the device RAM
{
  int i = 0, j = 0;

  LcdSend(HD44780_SET_CGRAM_ADD, lcd_CMD);//Позицианируем курсор на начало координат
  LcdSend(HD44780_SET_DDRAM_ADD, lcd_CMD);

  for (i = 0; i < (LCD_Y_RES >> 3); i++)        //грузим данные строками (было деление на 8)
    for (j = 0; j < LCD_X_RES; j++)        //грузим данные столюиками по 8 пикселей
    {
      LcdSend(LcdCache[((i * LCD_X_RES) + j)], lcd_DATA);       //вычисляем адрес в фрейм буфере, и данные от туда грузим в дисплей.
    }
}


void LcdClear(void)             //Clears the display
{
  int i = 0;

  for (i = 0; i < LCD_CACHSIZE; i++)
    LcdCache[i] = 0;            //забиваем всю память 0
  LcdUpdate();
}


void LcdClear_massive(void)     //Clears the display
{
  int i = 0;

  for (i = 0; i < LCD_CACHSIZE; i++)
    LcdCache[i] = 0;            //забиваем всю память 0
}


void LcdPixel(unsigned char x, unsigned char y, unsigned char mode)     //Displays a pixel at given absolute (x, y) location, mode -> Off, On or Xor
{
  int index = 0;
  unsigned char offset = 0, data = 0;

  if(x > LCD_X_RES)
    return;                     //если передали в функцию муть - выходим
  if(y > LCD_Y_RES)
    return;

  index = (((int) (y) >> 3) * Cntr_X_RES) + x;  //считаем номер байта в массиве памяти дисплея
  offset = y - ((y >> 3) << 3); //считаем номер бита в этом байте

  data = LcdCache[index];       //берем байт по найденному индексу

  if(mode == PIXEL_OFF)
    data &= (~(0x01 << offset));        //редактируем бит в этом байте
  else if(mode == PIXEL_ON)
    data |= (0x01 << offset);
  else if(mode == PIXEL_XOR)
    data ^= (0x01 << offset);

  LcdCache[index] = data;       //загружаем байт назад
}

void LcdLine(int x1, int y1, int x2, int y2, unsigned char mode)        //Draws a line between two points on the display - по Брезенхейму
{
  signed int dy = 0;
  signed int dx = 0;
  signed int stepx = 0;
  signed int stepy = 0;
  signed int fraction = 0;

  if(x1 > LCD_X_RES || x2 > LCD_X_RES || y1 > LCD_Y_RES || y2 > LCD_Y_RES)
    return;

  dy = y2 - y1;
  dx = x2 - x1;
  if(dy < 0)
  {
    dy = -dy;
    stepy = -1;
  } else
    stepy = 1;
  if(dx < 0)
  {
    dx = -dx;
    stepx = -1;
  } else
    stepx = 1;
  dy <<= 1;
  dx <<= 1;
  LcdPixel(x1, y1, mode);
  if(dx > dy)
  {
    fraction = dy - (dx >> 1);
    while (x1 != x2)
    {
      if(fraction >= 0)
      {
        y1 += stepy;
        fraction -= dx;
      }
      x1 += stepx;
      fraction += dy;
      LcdPixel(x1, y1, mode);
    }
  } else
  {
    fraction = dx - (dy >> 1);
    while (y1 != y2)
    {
      if(fraction >= 0)
      {
        x1 += stepx;
        fraction -= dy;
      }
      y1 += stepy;
      fraction += dx;
      LcdPixel(x1, y1, mode);
    }
  }
}


void LcdGotoXYFont(unsigned char x, unsigned char y)    //Sets cursor location to xy location. Range: 1,1 .. 14,6
{
  LcdCacheIdx = ((int) (y) - 1) * Cntr_X_RES + ((int) (x) - 1) * Cntr_Y_RES;
}

void clean_lcd_buf(void)        //очистка текстового буфера
{
  uint8_t i = 0;

  for (i = 0; i < 20; i++)
    lcd_buf[i] = 0;
}

void LcdChr(int ch)             //Displays a character at current cursor location and increment cursor location
{
  char i = 0;
  if(ch > 0x7f)
  {
    for (i = 0; i < 5; i++)
      LcdCache[LcdCacheIdx++] = lcd_font_table_rus[(ch * 5 + (i) - 0x3C0)];     //выделяем байт-столбик из символа и грузим в массив - 5 раз
  } else
  {
    for (i = 0; i < 5; i++)
      LcdCache[LcdCacheIdx++] = lcd_font_table[(ch * 5 + (i) - 0xA0)];  //выделяем байт-столбик из символа и грузим в массив - 5 раз
  }
  LcdCache[LcdCacheIdx++] = 0x00;       //добавляем пробел между символами
}

void LcdChrInv(int ch)          //Displays a character at current cursor location and increment cursor location
{
  unsigned char i = 0;
  if(ch > 0x7f)
  {
    for (i = 0; i < 5; i++)
      LcdCache[LcdCacheIdx++] = ~(lcd_font_table_rus[(ch * 5 + i - 0x3C0)]);    //выделяем байт-столбик из символа и грузим в массив - 5 раз
  } else
  {
    for (i = 0; i < 5; i++)
      LcdCache[LcdCacheIdx++] = ~(lcd_font_table[(ch * 5 + i - 0xA0)]); //выделяем байт-столбик из символа и грузим в массив - 5 раз
  }
  LcdCache[LcdCacheIdx++] = 0xFF;       //добавляем пробел между символами
}

void LcdString(unsigned char x, unsigned char y)        //Displays a string at current cursor location
{
  unsigned char i = 0;

  if(x > 17 || y > 8)
    return;
  LcdGotoXYFont(x, y);
  for (i = 0; i < 17; i++)
    if(lcd_buf[i])
      LcdChr(lcd_buf[i]);
  clean_lcd_buf();
}

void LcdStringInv(unsigned char x, unsigned char y)     //Displays a string at current cursor location
{
  unsigned char i = 0;

  if(x > 17 || y > 8)
    return;
  LcdGotoXYFont(x, y);
  for (i = 0; i < 17 - x; i++)
    if(lcd_buf[i])
      LcdChrInv(lcd_buf[i]);
  clean_lcd_buf();
}

void LcdChrBold(int ch)         //Displays a bold character at current cursor location and increment cursor location
{
  unsigned char i = 0;
  unsigned char a = 0, b = 0, c = 0;

  for (i = 0; i < 5; i++)
  {
    if(ch > 0x7f)
    {
      c = lcd_font_table_rus[(ch * 5 + i - 0x3C0)];     //выделяем столбец из символа
    } else
    {
      c = lcd_font_table[(ch * 5 + i - 0xA0)];  //выделяем столбец из символа
    }

    b = (c & 0x01) * 3;         //"растягиваем" столбец на два байта
    b |= (c & 0x02) * 6;
    b |= (c & 0x04) * 12;
    b |= (c & 0x08) * 24;

    c >>= 4;
    a = (c & 0x01) * 3;
    a |= (c & 0x02) * 6;
    a |= (c & 0x04) * 12;
    a |= (c & 0x08) * 24;

    LcdCache[LcdCacheIdx] = b;  //копируем байты в экранный буфер
    LcdCache[LcdCacheIdx + 1] = b;      //дублируем для получения жирного шрифта
    LcdCache[LcdCacheIdx + Cntr_X_RES] = a;
    LcdCache[LcdCacheIdx + Cntr_X_RES+1] = a;
    LcdCacheIdx = LcdCacheIdx + 2;
  }
  LcdCache[LcdCacheIdx++] = 0x00;       //для пробела между символами
  LcdCache[LcdCacheIdx++] = 0x00;
}

void LcdStringBold(unsigned char x, unsigned char y)    //Displays a string at current cursor location
{
  unsigned char i = 0;
  if(x > 17 || y > 8)
    return;
  LcdGotoXYFont(x, y);
  for (i = 0; i < 17 - x; i++)
    if(lcd_buf[i])
      LcdChrBold(lcd_buf[i]);
  clean_lcd_buf();
}

void LcdChrBig(int ch)          //Displays a character at current cursor location and increment cursor location
{
  unsigned char i = 0;
  unsigned char a = 0, b = 0, c = 0;

  for (i = 0; i < 5; i++)
  {
    if(ch > 0x7f)
    {
      c = lcd_font_table_rus[(ch * 5 + i - 0x3C0)];     //выделяем столбец из символа
    } else
    {
      c = lcd_font_table[(ch * 5 + i - 0xA0)];  //выделяем столбец из символа
    }

    b = (c & 0x01) * 3;         //"растягиваем" столбец на два байта
    b |= (c & 0x02) * 6;
    b |= (c & 0x04) * 12;
    b |= (c & 0x08) * 24;

    c >>= 4;
    a = (c & 0x01) * 3;
    a |= (c & 0x02) * 6;
    a |= (c & 0x04) * 12;
    a |= (c & 0x08) * 24;
    LcdCache[LcdCacheIdx] = b;
    LcdCache[LcdCacheIdx + Cntr_X_RES] = a;
    LcdCacheIdx = LcdCacheIdx + 1;
  }

  LcdCache[LcdCacheIdx++] = 0x00;
}

void LcdStringBig(unsigned char x, unsigned char y)     //Displays a string at current cursor location
{
  unsigned char i = 0;

  if(x > 17 || y > 8)
    return;
  LcdGotoXYFont(x, y);
  for (i = 0; i < 17 - x; i++)
    if(lcd_buf[i])
      LcdChrBig(lcd_buf[i]);
  clean_lcd_buf();
}

//////////////////////////////////////////////////////////////////////////////////////



void pulse_e() //импульс на вход Е индикатора
{
//    PIN_ON(PIN_E);
	//    delay_us(100);
	//    PIN_OFF(PIN_E);
	//    delay_us(39);

	HAL_GPIO_WritePin(Display_EN_GPIO_Port, Display_EN_Pin, GPIO_PIN_SET);//    PIN_ON(PIN_E);
	//HAL_Delay(1);
    HAL_GPIO_WritePin(Display_EN_GPIO_Port, Display_EN_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_E);
    //HAL_Delay(1);
}

void send_nibble(unsigned char data) //полубайтовый вывод
{
  //выводим половину байта на соответствующие контакты
  if(data & 0x01)
	HAL_GPIO_WritePin(Display_DB4_GPIO_Port, Display_DB4_Pin, GPIO_PIN_SET);//  PIN_ON(PIN_DB4);
  else
    HAL_GPIO_WritePin(Display_DB4_GPIO_Port, Display_DB4_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB4);

  if(data & 0x02)
		HAL_GPIO_WritePin(Display_DB5_GPIO_Port, Display_DB5_Pin, GPIO_PIN_SET);//  PIN_ON(PIN_DB5);
	  else
	    HAL_GPIO_WritePin(Display_DB5_GPIO_Port, Display_DB5_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB5);

  if(data & 0x04)
		HAL_GPIO_WritePin(Display_DB6_GPIO_Port, Display_DB6_Pin, GPIO_PIN_SET);//  PIN_ON(PIN_DB6);
	  else
	    HAL_GPIO_WritePin(Display_DB6_GPIO_Port, Display_DB6_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB6);

  if(data & 0x08)
		HAL_GPIO_WritePin(Display_DB7_GPIO_Port, Display_DB7_Pin, GPIO_PIN_SET);//  PIN_ON(PIN_DB7);
	  else
	    HAL_GPIO_WritePin(Display_DB7_GPIO_Port, Display_DB7_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB7);

  pulse_e(); //чтобы индикатор проглотил данные
}

void send_data(unsigned char data) //вывод 2хполбайта на индикатор
{
  //Первым шлем старшие полбайта: по инструкции
  //сначала столбец, потом строка таблицы знакогенератора
  send_nibble((data>>4) & 0x0F);
  send_nibble(data & 0x0F);
}

int check_busy_flag(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_PinState status;

	lcd44780_RW_1; // Read
	GPIO_InitStruct.Pin = Display_DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	lcd44780_E_1;
	status=HAL_GPIO_ReadPin(Display_DB7_GPIO_Port, Display_DB7_Pin);
	lcd44780_E_0;
	lcd44780_E_1;
	lcd44780_E_0;
	GPIO_InitStruct.Pin = Display_DB7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	lcd44780_RW_0;
	return status;
}

void init_LCD() //инициализация ЖК
{
	HAL_GPIO_WritePin(Display_Power_GPIO_Port, Display_Power_Pin, GPIO_PIN_RESET);
  //Выводы в 0
    HAL_GPIO_WritePin(Display_RS_GPIO_Port, Display_RS_Pin, GPIO_PIN_SET);//  PIN_OFF(PIN_RS); !!!
    HAL_GPIO_WritePin(Display_EN_GPIO_Port, Display_EN_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_E);
    HAL_GPIO_WritePin(Display_DB4_GPIO_Port, Display_DB4_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB4);
    HAL_GPIO_WritePin(Display_DB5_GPIO_Port, Display_DB5_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB5);
    HAL_GPIO_WritePin(Display_DB6_GPIO_Port, Display_DB6_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB6);
    HAL_GPIO_WritePin(Display_DB7_GPIO_Port, Display_DB7_Pin, GPIO_PIN_RESET);//  PIN_OFF(PIN_DB7);

	//Ждем включения ЖКИ
    HAL_Delay(500);
    lcd44780_RS_0;
    lcd44780_RW_0;
	send_nibble(0x02);
	send_nibble(0x02);
	send_nibble(0x08);// N=1 F=0

	while(check_busy_flag());

	send_nibble(0x00);
	send_nibble(0x0E);// D=1 C=1 B=0

	while(check_busy_flag());
	send_nibble(0x00);
	send_nibble(0x01);

	while(check_busy_flag());
	send_nibble(0x00);
	send_nibble(0x06); // I/D=1  S/H=0
//	send_nibble(0x04); // I/D=0  S/H=0

	while(check_busy_flag());

    lcd44780_RS_0;
    lcd44780_RW_0;
	send_nibble(0x02); // DL=0
	send_nibble(0x0A);// N=1 F=0 FT1=1 FT0=0
	while(check_busy_flag());


    send_data(0x1F); //переключение в графику
    while(check_busy_flag());

    send_data(0x01); //очистили от мусора ОЗУ (т.с. что clear())
    while(check_busy_flag());

    LcdClear_massive();

}


void init_LCD_pins() //Инициализация выводов МК для общения с ЖК
{
    lcd44780_RW_0;
	lcd44780_RS_0;
}

void lcd_write_byte(unsigned char addr, unsigned char data)
{

lcd44780_RS_0;
//HAL_Delay(1);
send_data(addr);
//HAL_Delay(1);
lcd44780_RS_1;
//HAL_Delay(1);
send_data(data);
//HAL_Delay(1);
while(check_busy_flag());

}

void LcdBarLine(uint32_t fill)    // рисуем прогресс-бар в второй строке
{
	uint16_t i, full_fill_position;
	float y;

	if(fill>0xFFFFF)return;

	// поиск свободного места в массиве кеша дисплея
//	for (i = LCD_CACHSIZE-1; i > LCD_X_RES; i--) // поиск свободного места в массиве кеша дисплея, в указанной строке
//		if(LcdCache[i]==0x00)
//			found_free_position=i;
	y=LCD_X_RES;
	y*=8; //сколько всего диступно места
	y/=(float)0xFFFFF;
	y*=(float)fill;// получаем коэфицент заполнения прогрессбара
	full_fill_position=floor(y/8);
	//need_to_be_filled=(LCD_CACHSIZE-1-found_free_position)*8 - ;
	for (i = LCD_X_RES; i < LCD_CACHSIZE; i++){ // заполнение прогрессбара
		if(y!=0)
		{
			if(full_fill_position>(i-LCD_X_RES))
			{
				LcdCache[i]=LcdCache[i]^0xFF;
			} else
			{
				y-=floor(y/8)*8;
				LcdCache[i]=LcdCache[i]^((1<<(uint16_t)y)-1);
				break;
			}
		}
	}

}


