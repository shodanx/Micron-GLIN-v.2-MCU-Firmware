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
void DAC_Write_FAST(void);

extern double DDS_clock_frequecny;
extern double DAC_fullrange_voltage;

extern double DDS_target_frequecny;
extern double DAC_target_speed;
extern uint32_t DDS_target_multipiller;

uint32_t DAC_tx_buffer;
uint16_t DAC_tx_tmp_buffer[2];


#endif
