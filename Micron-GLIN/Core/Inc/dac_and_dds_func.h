#ifndef __dac_and_dds_func_H
#define __dac_and_dds_func_H

#include <stdio.h>
#include <string.h>

//#include "main.h"
#include "spi.h"
//#include "function.h"

void DDS_Init(void);
void DDS_Update(void);
void DDS_Calculation(void);
__RAM_FUNC void DAC_SendInit(void);
void DAC_TEMP_CAL(void);
__RAM_FUNC void DAC_Write(uint32_t);
__RAM_FUNC void DAC_Write_FAST(void);

void Relay_control(uint8_t,uint8_t);
void CPLD_control(FunctionalState);

#endif
