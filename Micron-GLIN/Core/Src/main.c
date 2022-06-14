/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "circular_buffer.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
uint8_t OK[]="\r\n OK \n\rEnter command: ";
uint8_t Error2[]="\r\n Value out of range \n\r\n\rEnter command: ";
uint8_t Done[]="\r\n CYCLE COMPLETE ! \r\n";

char myLCDstr[32];
float temperature=10.23;
uint16_t tmpx;



//Calibration value
float DDS_clock_frequecny=1E7;
float DAC_fullrange_voltage;
float cal_DAC_up_voltage;
float cal_DAC_down_voltage;

float corr_coeff_1;
float corr_coeff_2;
float corr_coeff_3;

uint32_t DAC_tx_buffer;
uint16_t DAC_tx_tmp_buffer[2];
DAC_CONFIG1 cfg;

uint8_t CPLD_WORD=0x1;
float DDS_FTW=0;
float DDS_target_frequecny;
float DAC_target_speed;
uint32_t DDS_target_multipiller=1;

uint32_t DAC_code=0x0;
FunctionalState DAC_code_direction;

FunctionalState Need_update_DDS=0;
FunctionalState Ramp_dac_step_complete=0;

// #define CRICBUF_CLEAN_ON_POP
CIRC_GBUF_DEF(uint8_t, USB_rx_command_buffer, 30);
volatile FunctionalState USB_CDC_End_Line_Received;
uint8_t command_buffer[31];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Write_to_circ_buffer(uint8_t);

uint32_t EEPROM_read(uint32_t);
void EEPROM_write(uint32_t, uint32_t);
float binary_to_float(uint32_t);

void Parsing_command(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	cfg.PDN=0; // DAC power up
	cfg.VREFVAL=0x08; // 20 V ± 1.25 V
	cfg.FSET=1; // Enhanced THD
	cfg.DSDO=1; // SDO enabled
	cfg.ENALMP=0; // Indicates end of temperature calibration cycle. ALARM pin pulled low
	cfg.FSDO=0; // Fast SDO disabled
	cfg.LDACMODE=0; // DAC *NOT* updated on LDAC !!!!falling edge!!!!
	cfg.TNH_MASK=0x00; // This bit is writable only when FSET = 0
	cfg.EN_TMP_CAL=0; // Temperature calibration feature enabled

	DAC_target_speed=0.001; //  V/s
	DAC_code=0x7FFFF;
	DAC_code_direction=0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  init_LCD();


  Relay_control(1,1); // x1 mode
  Relay_control(2,0); // x2/x4 mode
  Relay_control(3,1); // Output Enable

  TMP117_Initialization(hi2c1);

	HAL_Delay(500); //WarmUP

	cal_DAC_up_voltage=binary_to_float(EEPROM_read(0x00)); // Read top voltage calibration from EEPROM in uV value
	cal_DAC_down_voltage=binary_to_float(EEPROM_read(0x08)); // Read top voltage calibration from EEPROM in uV value

	corr_coeff_1=binary_to_float(EEPROM_read(0x10));
	corr_coeff_2=binary_to_float(EEPROM_read(0x18));
	corr_coeff_3=binary_to_float(EEPROM_read(0x20));

	DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;

	HAL_Delay(250); //WarmUP
	DDS_Init();
	DAC_SendInit();

	//DAC_Write(DAC_code); //Middle
	//DAC_Write(0xFFFFF);
	DAC_Write(0x0);

	HAL_Delay(10);
	CDC_Transmit_FS(clear, strlen((const char *)clear));
	HAL_Delay(10);

	HAL_TIM_Base_Start_IT(&htim3);


	CPLD_WORD=0xA;
	CPLD_control(CPLD_WORD);
//	DDS_Update();
/*		while (1)
		{
			  tmpx=TMP117_get_Temperature(hi2c1);
			  temperature=tmpx*0.0078125;
			  sprintf(lcd_buf,"Temp: %.2f",temperature);
	          LcdString(1, 1);

			  sprintf(lcd_buf,"Hello world!");
	          LcdString(1, 2);
	          LcdUpdate();
		}
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		  tmpx=TMP117_get_Temperature(hi2c1);
		  temperature=tmpx*0.0078125;
		  sprintf(lcd_buf,"Temp: %.2f",temperature);
          LcdString(1, 1);

		  sprintf(lcd_buf,"Hello world!");
          LcdString(1, 2);
          LcdUpdate();

		if(USB_CDC_End_Line_Received)
		{
			uint8_t i=0;
			USB_CDC_End_Line_Received=0;
			while (1) {
				if (CIRC_GBUF_POP(USB_rx_command_buffer,&command_buffer[i])) command_buffer[i]='\n';
				if (command_buffer[i]=='\n' || command_buffer[i]=='\r') break;
				i++;
			}
			Parsing_command();
		}

		if(Need_update_DDS)
		{
			if(Ramp_dac_step_complete)
			{
				DDS_Update();
				Need_update_DDS=0;
				Ramp_dac_step_complete=0;
			}
			DDS_Calculation();

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
//==============================================================================================


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback and toggle LED
	if (htim == &htim3 )
	{
		Need_update_DDS=1;
	}
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
//==============================================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	DAC_Write_FAST();
	Ramp_dac_step_complete=1;

	if(GPIO_Pin == GPIO_PIN_9)
	{
		if(DAC_code_direction)
		{
			if(DAC_code<=(0xFFFFF-DDS_target_multipiller))
			{
				DAC_code+=DDS_target_multipiller;
				DAC_tx_buffer=0x01000000; // Write DAC-DATA
				DAC_tx_buffer+=(DAC_code & 0xFFFFF)<<4;

				DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
				DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);

			} else  {
				CPLD_control(0x0); // Disable LDAC signal

				cfg.LDACMODE=0;
				DAC_SendInit();
				CDC_Transmit_FS(Done, strlen((const char *)Done));  // SEND ERROR TO CDC!!!
				return;
			}
		}
		else
		{
			if(DAC_code>=DDS_target_multipiller)
			{
				DAC_code-=DDS_target_multipiller;
				DAC_tx_buffer=0x01000000; // Write DAC-DATA
				DAC_tx_buffer+=(DAC_code & 0xFFFFF)<<4;

				DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
				DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);

			} else {
				CPLD_control(0x0); // Disable LDAC signal
				cfg.LDACMODE=0;
				DAC_SendInit();
				CDC_Transmit_FS(Done, strlen((const char *)Done));  // SEND ERROR TO CDC!!!
				return;
			}
		}
	}
}

//==============================================================================================
void Write_to_circ_buffer(uint8_t Buf)
{
	if(CIRC_GBUF_PUSH(USB_rx_command_buffer, &Buf))	CIRC_GBUF_FLUSH(USB_rx_command_buffer); // If out of space, something wrong, clean all !!!
}

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
void Parsing_command(void)
{
	float atof_tmp;
	char *found;
	char decoded_string_1[31];
	char decoded_string_2[31];
	float dac_resolution;

	found = strtok((char *)command_buffer," ");
	if(found!=NULL)
	{
		strcpy(decoded_string_1,found);
	}
	else
	{
		HAL_Delay(10);
		CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
		HAL_Delay(10);
		return;
	}

	found = strtok(NULL,"\r");
	if(found!=NULL)
	{
		strcpy(decoded_string_2,found);
		for(int i=0;i<strlen(decoded_string_2);i++)if(decoded_string_2[i]==' ')decoded_string_2[i]='\0';
	}
	else
	{
		HAL_Delay(10);
		CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
		HAL_Delay(10);
		return;
	}
	// ==== SWEEP command ====
	if(!(strcmp(decoded_string_1,"SWEEP")))
	{
		if(!(strcmp(decoded_string_2,"START"))){
			DAC_TEMP_CAL();
			CPLD_control(CPLD_WORD); // Enable LDAC signal
			cfg.LDACMODE=1;
			DAC_SendInit();
			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"STOP"))){
				CPLD_control(0x0); // Disable LDAC signal
				cfg.LDACMODE=0;
				DAC_SendInit();
				HAL_Delay(10);
				CDC_Transmit_FS(OK, strlen((const char *)OK));
				HAL_Delay(10);
				return;
			}
			else
			{
				HAL_Delay(10);
				CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
				HAL_Delay(10);
				return;
			}

		}
	}
	// ==== DAC_SET command ====
	if(!(strcmp(decoded_string_1,"DAC_SET")))
	{
		if(!(strcmp(decoded_string_2,"TOP"))){
			DAC_code=0xFFFFF;
			DAC_code_direction=0;
			cfg.LDACMODE=0;
			DAC_SendInit();
			DAC_Write(DAC_code);
			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				DAC_code=0x0;
				DAC_code_direction=1;
				cfg.LDACMODE=0;
				DAC_SendInit();
				DAC_Write(DAC_code);
				HAL_Delay(10);
				CDC_Transmit_FS(OK, strlen((const char *)OK));
				HAL_Delay(10);
				return;
			}
			else
			{
				atof_tmp=atof(decoded_string_2);
				if(atof_tmp>=cal_DAC_down_voltage && atof_tmp<=cal_DAC_up_voltage)
				{
					dac_resolution=(cal_DAC_up_voltage-cal_DAC_down_voltage)/0xFFFFF; // Calculate 1 LSB resolution
					DAC_code=(uint32_t)((atof_tmp-cal_DAC_down_voltage)/dac_resolution);

					cfg.LDACMODE=0;
					DAC_TEMP_CAL();
					DAC_Write(DAC_code);

					HAL_Delay(10);
					CDC_Transmit_FS(OK, strlen((const char *)OK));
					HAL_Delay(10);
					return;
				}
				else
				{
					HAL_Delay(10);
					CDC_Transmit_FS(Error2, strlen((const char *)Error2));  // SEND ERROR TO CDC!!!
					HAL_Delay(10);
					return;
				}
			}

		}
	}

	// ==== DAC_CAL_TEMP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TEMP")))
	{
		if(!(strcmp(decoded_string_2,"START"))){
			DAC_code=0x7FFFF;
			DAC_Write(DAC_code);
			DAC_TEMP_CAL();
			DAC_Write(DAC_code);
			return;
		}
		else
		{
			HAL_Delay(10);
			CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!`
			HAL_Delay(10);
			return;
		}
	}

	// ==== DAC_CAL_POLY_A command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_A")))
	{
		atof_tmp=atof(decoded_string_2);
		corr_coeff_1=atof_tmp;
		EEPROM_write(0x10,float_to_binary(atof_tmp));
		HAL_Delay(10);
		CDC_Transmit_FS(OK, strlen((const char *)OK));
		HAL_Delay(10);
		return;
	}

	// ==== DAC_CAL_POLY_B command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_B")))
	{
		atof_tmp=atof(decoded_string_2);
		corr_coeff_2=atof_tmp;
		EEPROM_write(0x18,float_to_binary(atof_tmp));
		HAL_Delay(10);
		CDC_Transmit_FS(OK, strlen((const char *)OK));
		HAL_Delay(10);
		return;
	}

	// ==== DAC_CAL_POLY_C command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_C")))
	{
		atof_tmp=atof(decoded_string_2);
		corr_coeff_3=atof_tmp;
		EEPROM_write(0x20,float_to_binary(atof_tmp));
		HAL_Delay(10);
		CDC_Transmit_FS(OK, strlen((const char *)OK));
		HAL_Delay(10);
		return;
	}

	// ==== DAC_CAL_TOP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TOP")))
	{
		atof_tmp=atof(decoded_string_2);
		if((atof_tmp<10.1 && atof_tmp>9.9) || (atof_tmp>6.8 && atof_tmp<7.1))
		{
			cal_DAC_up_voltage=atof_tmp;
			EEPROM_write(0x00,float_to_binary(cal_DAC_up_voltage)); // Write top voltage calibration to EEPROM in uV value
			DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;

			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
		else
		{
			HAL_Delay(10);
			CDC_Transmit_FS(Error2, strlen((const char *)Error2));  // SEND ERROR TO CDC!!!
			HAL_Delay(10);
			return;
		}
	}


	// ==== DAC_CAL_DOWN command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_DOWN")))
	{
		atof_tmp=atof(decoded_string_2);
		if((atof_tmp>-10.1 && atof_tmp<-9.9) || (atof_tmp<-6.8 && atof_tmp>-7.1))
		{
			cal_DAC_down_voltage=atof_tmp;
			EEPROM_write(0x08,float_to_binary(cal_DAC_down_voltage)); // Write top voltage calibration to EEPROM in uV value
			DAC_fullrange_voltage=cal_DAC_up_voltage-cal_DAC_down_voltage;

			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
		else
		{
			HAL_Delay(10);
			CDC_Transmit_FS(Error2, strlen((const char *)Error2));  // SEND ERROR TO CDC!!!
			HAL_Delay(10);
			return;
		}
	}


	// ==== SWEEP_RATE command ====
	if(!(strcmp(decoded_string_1,"SWEEP_RATE")))
	{
		atof_tmp=atof(decoded_string_2);
		if(atof_tmp<0.001 || atof_tmp>1)
		{
			HAL_Delay(10);
			CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
			HAL_Delay(10);
			return;
		}
		else
		{
			DAC_target_speed=atof_tmp;

			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
	}

	// ==== SWEEP_DIRECTION command ====
	if(!(strcmp(decoded_string_1,"SWEEP_DIRECTION")))
	{
		if(!(strcmp(decoded_string_2,"UP"))){
			DAC_code_direction=1;
			HAL_Delay(10);
			CDC_Transmit_FS(OK, strlen((const char *)OK));
			HAL_Delay(10);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				DAC_code_direction=0;
				HAL_Delay(10);
				CDC_Transmit_FS(OK, strlen((const char *)OK));
				HAL_Delay(10);
				return;
			}
			else
			{
				HAL_Delay(10);
				CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
				HAL_Delay(10);
				return;
			}

		}
	}

	HAL_Delay(10);
	CDC_Transmit_FS(Error1, strlen((const char *)Error1));  // SEND ERROR TO CDC!!!
	HAL_Delay(10);
	return;
}

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


//==============================================================================================
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
