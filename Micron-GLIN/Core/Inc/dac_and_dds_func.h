#ifndef __dac_and_dds_func_H
#define __dac_and_dds_func_H

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "spi.h"

void DDS_Init(void);
void DDS_prepare_to_tempcal(void);
void DAC_SendInit(void);
void DAC_TEMP_CAL(void);
void DAC_Write(uint32_t);

extern float DDS_clock_frequecny;
extern float DAC_fullrange_voltage;

extern float DDS_target_frequecny;
extern float DAC_target_speed;
extern uint32_t DDS_target_multipiller;

#endif
