/*************************************************************************
 *    File name   : WS0010_GR.c
 *    Description : WS0010 graph driver
 *
 *    History :
 *    1. Date        : 12.07.2012
 *       Author      : S. Budakow
 *       Description : Create
 **************************************************************************/
#include "drv_hd44780.h"
#include "drv_hd44780_cnfg.h"
#include "drv_hd44780_l.h"

// Отсюда чудеса для WS0010
#define WS0010G_PosY         0x40    /* Set point to string 0/1*/
#define WS0010G_PosX         0x80    /* Set point to X */


#define HD44780_MAX_COMM_DLY          30      /* 3ms (tick 100us) */

// Font data for STDfontLCD_7pt
extern const FONT_INFO STDfontLCD_7ptFontInfo;
// Font data for Font_11x16
extern const FONT_INFO Font_11x16FontInfo;
// Font data for Font_4x6FontInfo
extern const FONT_INFO Font_4x6FontInfo;
// Font data for Font_8x16
extern const FONT_INFO Font_8x16FontInfo;
/*************************************************************************
 * Function Name: WS0010_Command
 * Parameters: Command
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Send command to WS0010
 *
 *************************************************************************/
HD44780_ERROR_CODE_DEF WS0010_Command (Int8U Command)
{
    HD44780WrComm(Command);
    return HD44780_BusyCheck(NULL,HD44780_MAX_COMM_DLY);
}

/*************************************************************************
 * Function Name: WS0010_Sw2GR
 * Parameters: Command
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Send command to WS0010
 *
 *************************************************************************/
HD44780_ERROR_CODE_DEF WS0010_Sw2GR (void)
{
    //переключение в графический режим
    WS0010_Command(0x08);
    WS0010_Command(0x1F);
    WS0010_Command(0x01);
    WS0010_Command(0x08|0x04);
    return HD44780_OK;
}

/*************************************************************************
 * Function Name: WS0010_Data
 * Parameters: Command
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Send Data to OLED
 *
 *************************************************************************/
HD44780_ERROR_CODE_DEF WS0010_Data (Int8U Data)
{

    HD44780WrData(Data);
    return HD44780_BusyCheck(NULL,HD44780_MAX_COMM_DLY);
}
/*************************************************************************
 * Function Name: WS0010_String_8
 * Parameters:
 * Int16U :PosX позиция по горизонтали в пикселах
 * Int16U  PosY позиция по вертикали в строках
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Show zero terminate string into OLED font Xx8
 *
 *************************************************************************/
HD44780_ERROR_CODE_DEF WS0010_String_8 (Int16U PosX, Int16U PosY, 
                                    const HD44780_STRING_DEF * DataStr,
                                    const FONT_INFO* Font)
{
HD44780_ERROR_CODE_DEF Error_code;
Int32U Xcount;
char* pnt;

    Error_code = HD44780_OK;
    //установка курсора в требуемую позицию начала строки
    //потом проверить на допустимость позиции на экране
    //CurX = PosX & 0x7F;
    //CurY = PosY & 0x01;
    Error_code = WS0010_Command(WS0010G_PosX | (PosX & 0x7F));
    if (Error_code != HD44780_OK)  return Error_code;
    Error_code =WS0010_Command(WS0010G_PosY | (PosY & 0x01)); 
    if (Error_code != HD44780_OK) return Error_code;
    while (*DataStr) {
        //указатель на начало пиксельной карты символа
        //(*DataStr - Font->startChar) - смещение относительно начального символа 
        //(Font->charInfo + (*DataStr - Font->startChar))->offset - смещение в таблице знакогенератора
        pnt = (char*)Font->data + (Font->charInfo + (*DataStr - Font->startChar))->offset;
        //(Font->charInfo + (*DataStr - Font->startChar))->widthBits - ширина текущего символа в пикселях
        for (Xcount = (Font->charInfo + (*DataStr - Font->startChar))->widthBits ; Xcount; Xcount--) {
            Error_code =WS0010_Data((Int8U)(*pnt));
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
        }
        //межсимвольный интервальчик
        for (Xcount = Font->spacePixels ; Xcount; Xcount--) {
            Error_code =WS0010_Data(0x00);
            if (Error_code != HD44780_OK)  return Error_code;
        }
        DataStr++;
    }
    return Error_code;
}
/*************************************************************************
 * Function Name: WS0010_String_16
 * Parameters:
 * Int16U :PosX позиция по горизонтали в пикселах
 * Int16U  PosY позиция по вертикали в строках
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Show zero terminate string into OLED font Xx16
 *
 *************************************************************************/
HD44780_ERROR_CODE_DEF WS0010_String_16 (Int16U PosX, Int16U PosY, 
                                    const HD44780_STRING_DEF * DataStr,
                                    const FONT_INFO* Font)
{
HD44780_ERROR_CODE_DEF Error_code;
Int16U CurX;
Int32U Xcount;
char* pnt;

    Error_code = HD44780_OK;
    //установка курсора в требуемую позицию начала строки
    CurX = PosX & 0x7F;
    while (*DataStr) {
        //(*DataStr - Font->startChar) - смещение относительно начального символа 
        //(Font->charInfo + (*DataStr - Font->startChar))->offset - смещение в таблице знакогенератора
        pnt = (char*)Font->data + (Font->charInfo + (*DataStr - Font->startChar))->offset;
        //верхняя полустрока
        Error_code = WS0010_Command(WS0010G_PosX | CurX);
        if (Error_code != HD44780_OK)  return Error_code;
        Error_code = WS0010_Command(WS0010G_PosY | 0x00); 
        if (Error_code != HD44780_OK)  return Error_code;
        //(Font->charInfo + (*DataStr - Font->startChar))->widthBits - ширина текущего символа в пикселях
        for (Xcount = (Font->charInfo + (*DataStr - Font->startChar))->widthBits ; Xcount; Xcount--) {
            Error_code = WS0010_Data((Int8U)(*pnt));
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
        }
        for (Xcount = Font->spacePixels ; Xcount; Xcount--) {
            Error_code =WS0010_Data(0x00);
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
        }
        Error_code = WS0010_Command(WS0010G_PosX | CurX);
        if (Error_code != HD44780_OK)  return Error_code;
        Error_code =WS0010_Command(WS0010G_PosY | 0x01); 
        if (Error_code != HD44780_OK)  return Error_code;
        pnt = (char*)Font->data + (Font->charInfo + (*DataStr - Font->startChar))->offset + (Font->charInfo + (*DataStr - Font->startChar))->widthBits;
        for (Xcount = (Font->charInfo + (*DataStr - Font->startChar))->widthBits ; Xcount; Xcount--) {
            Error_code = WS0010_Data((Int8U)(*pnt));
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
        }
        for (Xcount = Font->spacePixels ; Xcount; Xcount--) {
            Error_code = WS0010_Data(0x00);
            if (Error_code != HD44780_OK)  return Error_code;
        }
        CurX += (Font->charInfo + (*DataStr - Font->startChar))->widthBits + Font->spacePixels;
        DataStr++;
    }
#if 0
    старый вариант
    Error_code = HD44780_OK;
    //установка курсора в требуемую позицию начала строки
    CurX = PosX & 0x7F;
    while (*DataStr) {
        //(*DataStr - Font->startChar) - смещение относительно начального символа 
        //(Font->charInfo + (*DataStr - Font->startChar))->offset - смещение в таблице знакогенератора
        pnt = (char*)Font->data + (Font->charInfo + (*DataStr - Font->startChar))->offset;
        //верхняя полустрока
        pnt++;
        Error_code = WS0010_Command(WS0010G_PosX | CurX);
        if (Error_code != HD44780_OK)  return Error_code;
        Error_code = WS0010_Command(WS0010G_PosY | 0x00); 
        if (Error_code != HD44780_OK)  return Error_code;
        //(Font->charInfo + (*DataStr - Font->startChar))->widthBits - ширина текущего символа в пикселях
        for (Xcount = (Font->charInfo + (*DataStr - Font->startChar))->widthBits ; Xcount; Xcount--) {
            Error_code = WS0010_Data((Int8U)(*pnt));
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
            pnt++;
        }
        for (Xcount = Font->spacePixels ; Xcount; Xcount--) {
            Error_code =WS0010_Data(0x00);
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
        }
        Error_code = WS0010_Command(WS0010G_PosX | CurX);
        if (Error_code != HD44780_OK)  return Error_code;
        Error_code =WS0010_Command(WS0010G_PosY | 0x01); 
        if (Error_code != HD44780_OK)  return Error_code;
        pnt = (char*)Font->data + (Font->charInfo + (*DataStr - Font->startChar))->offset;
        for (Xcount = (Font->charInfo + (*DataStr - Font->startChar))->widthBits ; Xcount; Xcount--) {
            Error_code = WS0010_Data((Int8U)(*pnt));
            if (Error_code != HD44780_OK)  return Error_code;
            pnt++;
            pnt++;
        }
        for (Xcount = Font->spacePixels ; Xcount; Xcount--) {
            Error_code = WS0010_Data(0x00);
            if (Error_code != HD44780_OK)  return Error_code;
        }
        CurX += (Font->charInfo + (*DataStr - Font->startChar))->widthBits + Font->spacePixels;
        DataStr++;
    }
#endif
    return Error_code;
}
/*************************************************************************
 * Function Name: HD44780_StrShow в графическом режиме WS0010
 *                  для совместимости с ПО AI23
 * Parameters:
 * Int16U :PosX позиция по горизонтали
 * в знакоместах HD44780 для Y = 1 или 2 (совместимость ЕТМ)
 * в пикселях для остальных
 *          `
 * Int16U  PosY позиция по вертикали в строках HD44780
 *          если номер строки больше 2 - рисуем символы в графике с произвольной позиции Х
 *          3 - шрифт 11х16
 *          4 - шрифт 8х16
 *          10/11 - шрифт 4х6 первая и вторая строка
 *          20/21 - шрифт 6х8 первая и вторая строка
 * Return: MENU_ERROR_CODE_DEF
 *        HD44780_OK   0: Pass
 *      HD44780_ERROR  1: Busy check TimeOut
 * Description: Show zero terminate string into OLED font Xx16
 *
 *************************************************************************/
#ifdef WG0010_Graph_Mode

HD44780_ERROR_CODE_DEF HD44780_StrShow(HD44780_XY_DEF X, HD44780_XY_DEF Y, const HD44780_STRING_DEF * DataStr)
{

    switch (Y) {
    case 1:
    case 2:
    default:
        {
            //очистка первых двух пикселов, 
            //"стандартный" текст в графике выводится со 2-го по 50-й пиксел
            //всего 6*8=48 пикселов ширины
            if (X == 1) {
                WS0010_Command(WS0010G_PosX | 0x00);
                WS0010_Command(WS0010G_PosY | (Y-1));
                WS0010_Data(0);
                WS0010_Data(0);
            };
            WS0010_String_8(((X-1)*6+2),(Y-1),DataStr,&STDfontLCD_7ptFontInfo);
        }
        break;
    case 3:
        WS0010_String_16(X,0,DataStr,&Font_11x16FontInfo);
        break;
    case 4:
        WS0010_String_16(X,0,DataStr,&Font_8x16FontInfo);
        break;
    case 10:
        WS0010_String_8(X,0,DataStr,&Font_4x6FontInfo);
        break;
    case 11:
        WS0010_String_8(X,1,DataStr,&Font_4x6FontInfo);
        break;
    case 20:
        WS0010_String_8(X,0,DataStr,&STDfontLCD_7ptFontInfo);
        break;
    case 21:
        WS0010_String_8(X,1,DataStr,&STDfontLCD_7ptFontInfo);
        break;
    }
   return HD44780_OK;
}
#endif
