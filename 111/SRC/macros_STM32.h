//манипул€ции с битами
#ifndef __HOPPING_inc_macros_H__
#define __HOPPING_inc_macros_H__

#define True  1
#define False 0

#define _setL(port,bit) port->BSRR = GPIO_BSRR_BR##bit
#define _setH(port,bit) port->BSRR = GPIO_BSRR_BS##bit
#define _set(port,bit,val) _set##val(port,bit)
#define on(x) _set (x)
#define SET(x) _setH(x)

#define _clrL(port,bit) port->BSRR = GPIO_BSRR_BS##bit
#define _clrH(port,bit) port->BSRR = GPIO_BSRR_BR##bit
#define _clr(port,bit,val) _clr##val(port,bit)
#define off(x) _clr (x)
#define CLR(x) _clrH(x)

/*
#define _bitL(port,bit) (!(port&(1<<bit)))
#define _bitH(port,bit) (port&(1<<bit))
#define _bit(port,bit,val) _bit##val(port,bit)
#define active(x) _bit (x)
#define BIT _bitH

#define _cpl(port,bit,val) port^=(1<<bit)
#define cpl(x) _cpl (x)
#define CPL _cpl
*/
#endif

