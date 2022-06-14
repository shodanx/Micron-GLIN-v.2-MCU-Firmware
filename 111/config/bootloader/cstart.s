/****************************************************************************************
|  Description: bootloader C startup source file
|    File Name: cstart.s
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

        MODULE  ?cstartup

  ; Forward declaration of section.
  SECTION CSTACK:DATA:NOROOT(3)

	PUBLIC	EntryFromProg
	PUBLIC	reset_handler
	EXTERN	__cmain
	EXTERN  __vector_table
  EXTERN  ComSetConnectEntryState
  EXTERN ComSetDisconnectEntryState
        EXTWEAK __iar_init_core
        EXTWEAK __iar_init_vfp


/****************************************************************************************
** NAME:           EntryFromProg
** PARAMETER:      none
** RETURN VALUE:   none
** DESCRIPTION:    Called by the user program to activate the bootloader. The user 
**                 program can call this function from C in the following way:
**                         void ActivateBootloader(void)
**                         {
**                           void (*pEntryFromProgFnc)(void);
**
**                           pEntryFromProgFnc = (void*)0x08000150 + 1;
**                           pEntryFromProgFnc();
**                         }
**                 Note that the + 1 added to the function address is neccassary to
**                 enable a switch from Thumb2 to Thumb mode.
**
****************************************************************************************/
        SECTION .entry:CODE:REORDER(2)
        THUMB
EntryFromProg:	
  ; Initialize the stack pointer
  LDR	  R3, =sfe(CSTACK)
	MOV	  SP, R3

  BL	__iar_init_core
	BL	__iar_init_vfp
  /* this part makes the difference with the normal reset_handler */
  BL  ComSetConnectEntryState
	BL	__cmain


/****************************************************************************************
** NAME:           reset_handler
** PARAMETER:      none
** RETURN VALUE:   none
** DESCRIPTION:    Reset interrupt service routine. Configures the stack, initializes RAM
**                 and jumps to function main.
**
****************************************************************************************/
        SECTION .text:CODE:REORDER(2)
        THUMB
reset_handler:	
  ; Initialize the stack pointer
  LDR	  R3, =sfe(CSTACK)
	MOV	  SP, R3

  BL	__iar_init_core
	BL	__iar_init_vfp
  /* this part makes the difference with function EntryFromProg */
  BL  ComSetDisconnectEntryState
	BL	__cmain

	REQUIRE __vector_table
	

        END

/************************************ end of cstart.s **********************************/
