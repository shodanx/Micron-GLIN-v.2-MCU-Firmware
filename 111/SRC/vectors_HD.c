/****************************************************************************************
|  Description: bootloader interrupt vector table source file
|    File Name: vectors.c
|
|----------------------------------------------------------------------------------------
|                          C O P Y R I G H T
|----------------------------------------------------------------------------------------
|   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
|
|----------------------------------------------------------------------------------------
|                            L I C E N S E
|----------------------------------------------------------------------------------------
| This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
| modify it under the terms of the GNU General Public License as published by the Free
| Software Foundation, either version 3 of the License, or (at your option) any later
| version.
|
| OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
| without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
| PURPOSE. See the GNU General Public License for more details.
|
| You should have received a copy of the GNU General Public License along with OpenBLT.
| If not, see <http://www.gnu.org/licenses/>.
|
| A special exception to the GPL is included to allow you to distribute a combined work 
| that includes OpenBLT without being obliged to provide the source code for any 
| proprietary components. The exception text is included at the bottom of the license
| file <license.html>.
| 
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "AI23_inc.h"                                  /* generic header                 */


/****************************************************************************************
* External functions
****************************************************************************************/
extern void __iar_program_start( void );


/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef union
{
  void (*func)(void);                                 /* for ISR function pointers     */
  void *ptr;                                          /* for stack pointer entry       */
}tIsrFunc;                                            /* type for vector table entries */


/****************************************************************************************
** NAME:           UnusedISR
** PARAMETER:      none
** RETURN VALUE:   none
** DESCRIPTION:    Catch-all for unused interrrupt service routines.
**
****************************************************************************************/
void UnusedISR(void)
{
  /* unexpected interrupt occured, so halt the system */
  while (1) { ; }
} /*** end of UnusedISR ***/


/****************************************************************************************
*                I N T E R R U P T     V E C T O R     T A B L E
****************************************************************************************/
#pragma language=extended                             /* enable IAR extensions         */
#pragma segment="CSTACK"

__root const tIsrFunc __vector_table[] @ ".intvec" =
{
  { .ptr = __sfe( "CSTACK" )       },                 /* the initial stack pointer     */
  { &__iar_program_start           },                 /* the reset handler             */
  { UnusedISR                      },                 /* NMI Handler                   */
  { UnusedISR                      },                 /* Hard Fault Handler            */
  { UnusedISR                      },                 /* MPU Fault Handler             */
  { UnusedISR                      },                 /* Bus Fault Handler             */
  { UnusedISR                      },                 /* Usage Fault Handler           */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* SVCall Handler                */
  { UnusedISR                      },                 /* Debug Monitor Handler         */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* PendSV Handler                */
  { SysTick_Handler                },                 /* SysTick Handler               */
  { UnusedISR                      },                 /* Window Watchdog               */
  { UnusedISR                      },                 /* PVD through EXTI Line detect  */
  { UnusedISR                      },                 /* Tamper                        */
  { UnusedISR                      },                 /* RTC                           */
  { UnusedISR                      },                 /* Flash                         */
  { UnusedISR                      },                 /* RCC                           */
  { UnusedISR                      },                 /* EXTI Line 0                   */
  { UnusedISR                      },                 /* EXTI Line 1                   */
  { UnusedISR                      },                 /* EXTI Line 2                   */
  { UnusedISR                      },                 /* EXTI Line 3                   */
  { UnusedISR                      },                 /* EXTI Line 4                   */
  { UnusedISR                      },                 /* DMA1 Channel 1                */
  { UnusedISR                      },                 /* DMA1 Channel 2                */
  { UnusedISR                      },                 /* DMA1 Channel 3                */
  { UnusedISR                      },                 /* DMA1 Channel 4                */
  { UnusedISR                      },                 /* DMA1 Channel 5                */
  { UnusedISR                      },                 /* DMA1 Channel 6                */
  { UnusedISR                      },                 /* DMA1 Channel 7                */
  { UnusedISR                      },                 /* ADC1 and ADC2                 */
  { USB_HP_CAN1_TX_IRQHandler      },                 /* CAN1 TX                       */
  { USB_LP_CAN1_RX0_IRQHandler     },                 /* CAN1 RX0                      */
  { UnusedISR                      },                 /* CAN1 RX1                      */
  { UnusedISR                      },                 /* CAN1 SCE                      */
  { UnusedISR                      },                 /* EXTI Line 9..5                */
  { UnusedISR                      },                 /* TIM1 Break                    */
  { UnusedISR                      },                 /* TIM1 Update                   */
  { UnusedISR                      },                 /* TIM1 Trigger and Commutation  */
  { UnusedISR                      },                 /* TIM1 Capture Compare          */
  { TIM2_IRQHandler                },                 /* TIM2                          */
  { UnusedISR                      },                 /* TIM3                          */
  { UnusedISR                      },                 /* TIM4                          */
  { UnusedISR                      },                 /* I2C1 Event                    */
  { UnusedISR                      },                 /* I2C1 Error                    */
  { UnusedISR                      },                 /* I2C2 Event                    */
  { UnusedISR                      },                 /* I2C1 Error                    */
  { UnusedISR                      },                 /* SPI1                          */
  { UnusedISR                      },                 /* SPI2                          */
  { USART1_IRQHandler              },                 /* USART1                        */
  { UnusedISR                      },                 /* USART2                        */
  { UnusedISR                      },                 /* USART3                        */
  { UnusedISR                      },                 /* EXTI Line 15..10              */
  { UnusedISR                      },                 /* RTC alarm through EXTI line   */
  { UnusedISR                      },                 /* USB OTG FS Wakeup             */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* Reserved                      */
  { UnusedISR                      },                 /* TIM5                          */
  { UnusedISR                      },                 /* SPI3                          */
  { UnusedISR                      },                 /* UART4                         */
  { UnusedISR                      },                 /* UART5                         */
  { UnusedISR                      },                 /* TIM6                          */
  { UnusedISR                      },                 /* TIM7                          */
  { UnusedISR                      },                 /* DMA2 Channel1                 */
  { UnusedISR                      },                 /* DMA2 Channel2                 */
  { UnusedISR                      },                 /* DMA2 Channel3                 */
  { UnusedISR                      },                 /* DMA2 Channel4                 */
  { UnusedISR                      },                 /* DMA2 Channel5                 */
  { UnusedISR                      },                 /* Ethernet                      */
  { UnusedISR                      },                 /* Ethernet Wakeup               */
  { UnusedISR                      },                 /* CAN2 TX                       */
  { UnusedISR                      },                 /* CAN2 RX0                      */
  { UnusedISR                      },                 /* CAN2 RX1                      */
  { UnusedISR                      },                 /* CAN2 SCE                      */
  { UnusedISR                      },                 /* USB OTG FS                    */
  { .ptr = (void*)0x55AA11EE       },                 /* Reserved for OpenBLT checksum */
};


/************************************ end of hw.c **************************************/


