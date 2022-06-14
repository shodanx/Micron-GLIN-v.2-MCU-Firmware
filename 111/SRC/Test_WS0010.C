//-------------------------------------------------------------------------------------------------
// SeregaB (2012)
// (c) Rados¦aw Kwiecieё, radek@dxp.pl
// http://en.radzio.dxp.pl
//-------------------------------------------------------------------------------------------------
#include "WS0010_inc.h"

extern signed int __ICFEDIT_intvec_start__;

void DelayResolution100us(Int32U Dly);

int main(void)
{
//Int8U x;
//Int8U* pnt;
//Int16U y;

    SystemInit();
    // Vector Table Relocation in Internal FLASH.
    SCB->VTOR = (uint32_t)&(__ICFEDIT_intvec_start__);
    HD44780_PowerUpInit();
//    if (SysTick_Config(SystemCoreClock / 1000))	{
//      /* Capture error */
//      while (1);
//    }
    //WS0010_Command(0x08);
    //WS0010_Command(0x17);
    //WS0010_Command(0x01);
    //WS0010_Command(0x08|0x04);

    //HD44780_StrShow(1, 1,  "*Проверка*");
    //HD44780_StrShow(1, 2,  "0123456789");

    //переключение в графический режим
    WS0010_Sw2GR();
    /*
    WS0010_Command(0x08);
    WS0010_Command(0x1F);
    WS0010_Command(0x01);
    WS0010_Command(0x08|0x04); 
    */ 
    WS0010_Command(0x01);
    HD44780_StrShow(1, 1,  "А-Я12345");
    HD44780_StrShow(1, 2,  "а я67890");
    WS0010_Command(0x01);
    HD44780_StrShow(1, 1,  "A-Z Test");
    HD44780_StrShow(1, 2,  "a-z TEST");
    WS0010_Command(0x01);
    HD44780_StrShow(0, 10,  "АБВГДЕЖЗИК");
    HD44780_StrShow(0, 11,  "абвгдежзик");
    //высоту делать так   
    HD44780_StrShow(0, 4,  "1234");
    HD44780_StrShow(36, 10,  "760 ");
    HD44780_StrShow(36, 11,  "QFE ");
    
    HD44780_StrShow(0, 4,  "5678");
    HD44780_StrShow(36, 10,  "123 ");
    HD44780_StrShow(36, 11,  "QNH ");

    HD44780_StrShow(0, 4,  "9012");
    HD44780_StrShow(36, 10,  "458 ");
    HD44780_StrShow(36, 11,  "QNE ");

    HD44780_StrShow(0, 4,  "####");
    HD44780_StrShow(36, 10,  "XXX ");
    HD44780_StrShow(36, 11,  "QFE ");

    
    HD44780_StrShow(1, 2,  "12345678");
    HD44780_StrShow(1, 1,  "Проверка");

    HD44780_StrShow(0, 3,  "12345");
    HD44780_StrShow(0, 4,  "12345");
    WS0010_Command(0x01);
    //скорость делать так
    WS0010_Command(0x01);
    HD44780_StrShow(0, 3,  "080");
    HD44780_StrShow(36, 20,  "км ");
    HD44780_StrShow(36, 21,  "/ч ");

    WS0010_Command(0x01);
    HD44780_StrShow(0, 3,  "080");
    HD44780_StrShow(36, 10,  "   ");
    HD44780_StrShow(36, 11,  "узл ");
    WS0010_Command(0x01);
    HD44780_StrShow(0, 3,  "080");
    HD44780_StrShow(36, 10,  "   ");
    HD44780_StrShow(36, 11,  "knt ");
    HD44780_StrShow(1, 1,  "12345678");
    HD44780_StrShow(1, 2,  "12345678");

    // вариометр показывать так
    WS0010_Command(0x01);
    HD44780_StrShow(0, 3,  "('&')");// заход
    HD44780_StrShow(0, 3,  "(1&2)");//положительные
    HD44780_StrShow(0, 3,  "$2&3)");//отрицательные
    HD44780_StrShow(0, 3,  "(45()");//положительные > 9,9
    HD44780_StrShow(0, 3,  "$67()");//отрицательные <-9.9

    HD44780_StrShow(1, 1,  "12345678");
    HD44780_StrShow(1, 2,  "12345678");

    HD44780_StrShow(1, 1,  "\3\4\7 \4\5   ");
    HD44780_StrShow(1, 2,  "100\6С        ");

#if 0
    WS0010_Command(0x01);
    WS0010_String_16(0,0,"12:45",&Font_12x16FontInfo);

    WS0010_Command(0x01);
    
    WS0010_String_16(0,0,"12:45",&Font_12x16FontInfo);
    WS0010_String_16(0,0,"2345",&Font_12x16FontInfo);
#endif
#if 0
    WS0010_String_8(0,0,"12345678",&courierNew_8ptFontInfo);
    WS0010_String_8(0,1,"87654321",&courierNew_8ptFontInfo);

    WS0010_String_8(0,0,"12345678",&STDfontLCD_7ptFontInfo);
    WS0010_String_8(0,1,"*Testig*",&STDfontLCD_7ptFontInfo);
     
    WS0010_String_8(0,0,"Проверка",&STDfontLCD_7ptFontInfo);
    WS0010_String_8(0,1,"ЯяКкРрЮю",&STDfontLCD_7ptFontInfo);

    WS0010_String_16(0,0,"12:45",&franklinGothicMedium_16ptFontInfo);
    WS0010_String_16(0,0,"2345",&verdana_16ptFontInfo);
 
    WS0010_String_8(25,0,"Часы ",&STDfontLCD_7ptFontInfo);
    WS0010_String_8(25,1,"Мин. ",&STDfontLCD_7ptFontInfo);
#endif
#if 0
    WS0010_Command(0x01);
    WS0010_String_16(0,0,"12",&arial_16ptFontInfo);
    WS0010_String_8(27,0,"Часы ",&STDfontLCD_7ptFontInfo);
    WS0010_String_8(23,1,"3456 ",&STDfontLCD_7ptFontInfo);


    WS0010_Command(0x01);
    WS0010_String_16(0,0,"12",&arial_16ptFontInfo);
    WS0010_String_8(27,0,"Часы ",&STDfontLCD_7ptFontInfo);

    HD44780_StrShow(25, 11,  "34.56 ");
    //WS0010_String_8(24,1,":34:56 ",&STDfontLCD_7ptFontInfo);
#endif
#if 0
    WS0010_Command(0x40 + 0);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&arial_8ptBitmaps;
    for (y=0;y<8;y++){
        for (x=0;x<5; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
        }
        WS0010_Data(0);
    }

    WS0010_Command(0x40 + 1);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&courierNew_8ptBitmaps ;
    for (y=0;y<8;y++){
        for (x=0;x<5; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
        }
        WS0010_Data(0);
    }

    WS0010_Command(0x01);

    WS0010_Command(0x40 + 0);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&verdana_16ptBitmaps;
    pnt++;
    for (y=0;y<4;y++){
        for (x=0;x<11; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }
    WS0010_Command(0x40 + 1);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&verdana_16ptBitmaps;
    for (y=0;y<4;y++){
        for (x=0;x<11; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }

    WS0010_Command(0x01);

    WS0010_Command(0x40 + 0);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&liberationSansNarrow_16ptBitmaps;
    pnt++;
    for (y=0;y<5;y++){
        for (x=0;x<9; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }
    WS0010_Command(0x40 + 1);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&liberationSansNarrow_16ptBitmaps;
    for (y=0;y<5;y++){
        for (x=0;x<9; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }

    WS0010_Command(0x01);
    WS0010_Command(0x40 + 0);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&franklinGothicMedium_16ptBitmaps;
    pnt +=20;
    pnt++;
    for (y=0;y<5;y++){
        for (x=0;x<10; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }
    WS0010_Command(0x40 + 1);
    WS0010_Command(0x80 + 0);
    pnt = (Int8U*)&franklinGothicMedium_16ptBitmaps;
    pnt +=20;
    for (y=0;y<5;y++){
        for (x=0;x<10; x++) {
            WS0010_Data((Int8U)(*pnt));
            pnt++;
            pnt++;
        }
        WS0010_Data(0);
    }

    y=0x01;
    WS0010_Command(0x40 + 1);
    WS0010_Command(0x80 + 0);
    for (x=0;x<50; x++) {
        WS0010_Data((Int8U)(y&0x00FF));
        if ((x & 0x08) == 0) y <<= 1;
        else          y >>= 1;

    }

    //переключение в текстовый режим
    WS0010_Command(0x08);
    WS0010_Command(0x17);
    WS0010_Command(0x01);
    WS0010_Command(0x04 | 0x08);
    HD44780_StrShow(1, 1,  "Проверка");
    HD44780_StrShow(1, 2,  "Test LCD");
#endif
    for( ; ; );
}
//-------------------------------------------------------------------------------------------------
//
//-------------------------------------------------------------------------------------------------

#define DLY_100US  450
/*************************************************************************
 * Function Name: DelayResolution100us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 100us)
 *
 *************************************************************************/
void DelayResolution100us(Int32U Dly)
{
  for(; Dly; Dly--)
  {
    for(volatile Int32U j = DLY_100US; j; j--)
    {
    }
  }
} 

