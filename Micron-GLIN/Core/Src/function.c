#include "stm32l1xx_hal.h"
#include "function.h"
#include "usb_device.h"
#include "circular_buffer.h"

extern void DDS_Init(void);
extern void DDS_Update(void);
extern void DDS_Calculation(void);
extern void DAC_SendInit(void);
extern void DAC_TEMP_CAL(void);
extern void DAC_Write(uint32_t);
extern void DAC_Write_FAST(void);

extern void Relay_control(uint8_t,uint8_t);
extern void CPLD_control(uint8_t);

extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);

extern uint8_t CPLD_WORD;
extern FunctionalState DAC_code_direction;
extern float DAC_fullrange_voltage;
extern float cal_DAC_up_voltage;
extern float cal_DAC_down_voltage;
extern float corr_coeff_1;
extern float corr_coeff_2;
extern float corr_coeff_3;



volatile FunctionalState USB_CDC_End_Line_Received;
uint8_t command_buffer[31];

uint8_t OK[]="OK\n\r";
uint8_t run_cal[]="\r\nCalibration in progress..";
uint8_t clear[]="\033c \r\n";
uint8_t Error1[]="\033c \r\n ERROR command not recognized \n\r\n\r"
		"\n\r"
		"Hello dear ampnuts!\n\r"
		"I'm Micron-GLIN, please tell me what you want?\n\r"
		"\n\r"
		"Usage:\n\r"
		"SWEEP START/STOP         - control sweep cycle\n\r"
		"SWEEP_RATE 1.0E-3        - set dv/dt speed (range 1...0.001)\n\r"
		"SWEEP_DIRECTION UP/DOWN  - set dv/dt direction(increase or decrease)\n\r"
		"DAC_SET TOP/DOWN/3E.4567 - set DAC to 0xFFFFF, 0x0 or exact voltage value\n\r"

		"\n\r"
		"DAC_CAL_TOP 10.01234     - set maximum positive DAC voltage\n\r"
		"DAC_CAL_DOWN -9.99876    - set maximum negative DAC voltage\n\r"
		"DAC_CAL_TEMP START       - start DAC temperature calibration cycle\n\r"
		"\n\r"
		"DAC_CAL_POLY_A 1.266415E-16 - set Linearity correction\n\r" //1.266415E-16x2 - 1.845382E-10x + 1.000056E+00
		"DAC_CAL_POLY_B 1.845382E-10 - set Linearity correction\n\r"
		"DAC_CAL_POLY_C 1.000056E+00 - set Linearity correction\n\r"
		"\n\r"
		"Enter command: ";
uint8_t OK_Enter[]="\r\n OK \n\rEnter command: ";
uint8_t Error2[]="\r\n Value out of range \n\r\n\rEnter command: ";
uint8_t Done[]="\r\n CYCLE COMPLETE ! \r\n";




void send_answer_to_CDC(uint8_t type)
{
	switch(type)
	{
	case ERROR_TYPE_1:
		HAL_Delay(10);
		CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
		HAL_Delay(10);
		break;
	case ERROR_TYPE_2:
		HAL_Delay(10);
		CDC_Transmit_FS(Error2, strlen((const char *)Error2));  // SEND ERROR TO CDC!!!
		HAL_Delay(10);
		break;
	case OK_TYPE_1:
		HAL_Delay(10);
		CDC_Transmit_FS(OK, strlen((const char *)OK));
		HAL_Delay(10);
		break;
	case OK_TYPE_2:
		HAL_Delay(10);
		CDC_Transmit_FS(OK_Enter, strlen((const char *)OK_Enter));
		HAL_Delay(10);
		break;
	case CLEAR_TYPE_1:
		HAL_Delay(10);
		CDC_Transmit_FS(clear, strlen((const char *)clear));
		HAL_Delay(10);
		break;
	case DONE_TYPE_1:
		HAL_Delay(10);
		CDC_Transmit_FS(Done, strlen((const char *)Done));
		HAL_Delay(10);
		break;
	case RUN_CAL_TYPE_TEMP:
		HAL_Delay(10);
		CDC_Transmit_FS(run_cal, strlen((const char *)run_cal));
		HAL_Delay(10);
		break;
	}
}
void cmd_SWEEP_START()
{
	DAC_TEMP_CAL();
	CPLD_control(CPLD_WORD); // Enable LDAC signal
	DAC_SendInit();
}

/////////////////////////////////////////////////////////
void cmd_SWEEP_STOP()
{
	CPLD_control(0x0); // Disable LDAC signal
	DAC_SendInit();
}

/////////////////////////////////////////////////////////
void cmd_DAC_SET(uint32_t code)
{
	if (code>0xFFFFF)return;

	switch(code)
	{
	case DAC_CODE_DOWN:
		DAC_code_direction=1;
		break;
	case DAC_CODE_TOP:
		DAC_code_direction=0;
		break;
	}

	CPLD_control(0x0); // Disable LDAC signal
	DAC_SendInit();
	DAC_TEMP_CAL();
	DAC_Write(code);
}


/////////////////////////////////////////////////////////
FunctionalState cmd_CAL(uint8_t cmd, float coeff)
{
	float tmpx;

	tmpx=coeff;

	switch(cmd)
	{
	case DAC_CAL_TEMP:
		DAC_Write(DAC_CODE_MIDDLE);
		DAC_TEMP_CAL();
		DAC_Write(DAC_CODE_MIDDLE);
		break;
	case DAC_CAL_POLY_A:
		EEPROM_write(corr_coeff_1_EEPROM_ADDRESS,float_to_binary(tmpx));
		break;
	case DAC_CAL_POLY_B:
		EEPROM_write(corr_coeff_2_EEPROM_ADDRESS,float_to_binary(tmpx));
		break;
	case DAC_CAL_POLY_C:
		EEPROM_write(corr_coeff_3_EEPROM_ADDRESS,float_to_binary(tmpx));
		break;
	case DAC_CAL_TOP:
		if((tmpx<10.1 && tmpx>9.9) || (tmpx>6.8 && tmpx<7.1))
		{
			cal_DAC_up_voltage=tmpx;
			EEPROM_write(cal_DAC_up_voltage_EEPROM_ADDRESS,float_to_binary(tmpx)); // Write top voltage calibration to EEPROM in uV value
			DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;
		}
		else return 0;
		break;
	case DAC_CAL_DOWN:
		if((tmpx>-10.1 && tmpx<-9.9) || (tmpx<-6.8 && tmpx>-7.1))
		{
			cal_DAC_down_voltage=tmpx;
			EEPROM_write(cal_DAC_down_voltage_EEPROM_ADDRESS,float_to_binary(tmpx)); // Write top voltage calibration to EEPROM in uV value
			DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;
		}
		else return 0;
		break;
	}
	return 1;

}
/////////////////////////////////////////////////////////
void load_data_from_EEPROM(void)
{
	cal_DAC_up_voltage=binary_to_float(EEPROM_read(cal_DAC_up_voltage_EEPROM_ADDRESS)); // Read top voltage calibration from EEPROM in uV value
	cal_DAC_down_voltage=binary_to_float(EEPROM_read(cal_DAC_down_voltage_EEPROM_ADDRESS)); // Read top voltage calibration from EEPROM in uV value
	DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;

	corr_coeff_1=binary_to_float(EEPROM_read(corr_coeff_1_EEPROM_ADDRESS));
	corr_coeff_2=binary_to_float(EEPROM_read(corr_coeff_2_EEPROM_ADDRESS));
	corr_coeff_3=binary_to_float(EEPROM_read(corr_coeff_3_EEPROM_ADDRESS));
}
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


//==============================================================================================
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
#pragma GCC push_options
#pragma GCC optimize ("O0")

float binary_to_float(uint32_t a)
{
	    int * p;
	    float out=0;

	    p = &out;
	    (*p)=a;
	    return out;
}


uint32_t float_to_binary(float a)
{
	    int i;
	    int * p;
	    uint32_t out=0;

	    p = &a;
	    for (i = sizeof(int) * 8 - 1; i >= 0; i--)
	    {
	    	out+=((*p) >> i & 1)<<i;
	    }

	    return out;
}
#pragma GCC pop_options
#pragma GCC diagnostic pop
//==============================================================================================


//==============================================================================================
uint32_t EEPROM_read(uint32_t address_of_read)
{
	uint32_t Address;

	/*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by
     DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */
	Address = DATA_EEPROM_START_ADDR + address_of_read;
	if(Address > DATA_EEPROM_END_ADDR)
	{
		return 0x00;
	}
	return *(__IO uint32_t *) Address;
}
//==============================================================================================
void EEPROM_write(uint32_t address_of_read, uint32_t data)
{
	uint32_t Address;
	HAL_StatusTypeDef FLASHStatus;

	/* Clear all pending flags */
	//FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);

	/*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by
	     DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */
	Address = DATA_EEPROM_START_ADDR + address_of_read;
	if(Address > DATA_EEPROM_END_ADDR)
	{
		return;
	}

	HAL_FLASHEx_DATAEEPROM_Unlock();
	FLASHStatus = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, Address, data);
	HAL_FLASHEx_DATAEEPROM_Lock();

	if(FLASHStatus != HAL_OK)
	{
		return;
	}
	//FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);

	if(*(__IO uint32_t *) Address != data)
	{
		return;
	}

}
