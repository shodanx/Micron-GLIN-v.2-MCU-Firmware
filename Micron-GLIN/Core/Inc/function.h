/*
 * function.h
 *
 *  Created on: 17 июн. 2022 г.
 *      Author: User
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_

#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <string.h>


#define ERROR_TYPE_1	0
#define ERROR_TYPE_2	1
#define OK_TYPE_1		2
#define OK_TYPE_2		3
#define CLEAR_TYPE_1	4
#define DONE_TYPE_1		5
#define DAC_CAL_TEMP	6
#define DAC_CAL_POLY_A	7
#define DAC_CAL_POLY_B	8
#define DAC_CAL_POLY_C	9
#define DAC_CAL_TOP		10
#define DAC_CAL_DOWN	11
#define GAIN_X2_CAL		12
#define GAIN_X4_CAL		13
#define NONE			14
#define RUN_CAL_TYPE_TEMP 15

#define Output_off_STATE	0
#define Output_x1_STATE		1
#define Output_x2_STATE		2
#define Output_x4_STATE		3

#define NO_UPDATE_STATE			0
#define NEED_UPDATE_CPLD_STATE	1

#define dU_dt_SCREEN		0
#define Hello_SCREEN		1
#define Warm_up_SCREEN		2
#define Ready_SCREEN		3

#define DAC_CODE_TOP	0xFFFFF
#define DAC_CODE_DOWN	0x0
#define DAC_CODE_MIDDLE	0x7FFFF

#define DATA_EEPROM_START_ADDR     			0x08080000
#define DATA_EEPROM_END_ADDR       			0x080827FF
#define DATA_EEPROM_PAGE_SIZE      			0x8

#define cal_DAC_up_voltage_EEPROM_ADDRESS 	0x0000
#define cal_DAC_down_voltage_EEPROM_ADDRESS 0x0008
#define corr_coeff_1_EEPROM_ADDRESS 		0x0010
#define corr_coeff_2_EEPROM_ADDRESS 		0x0018
#define corr_coeff_3_EEPROM_ADDRESS 		0x0020
#define gain_x2_EEPROM_ADDRESS				0x0028
#define gain_x4_EEPROM_ADDRESS				0x0030

void send_answer_to_CDC(uint8_t);
void cmd_SWEEP_START();
void cmd_SWEEP_STOP();
void cmd_DAC_SET(uint32_t);
FunctionalState cmd_SET_OUTPUT_VOLTAGE(float);
FunctionalState cmd_SWEEP_RATE(float);

FunctionalState cmd_CAL(uint8_t, float);

void display_screen(uint8_t);
void output_state(uint8_t);


uint32_t float_to_binary(float);
float binary_to_float(uint32_t);

uint32_t EEPROM_read(uint32_t);
void EEPROM_write(uint32_t, uint32_t);
float binary_to_float(uint32_t);
void load_data_from_EEPROM(void);

#endif /* INC_FUNCTION_H_ */
