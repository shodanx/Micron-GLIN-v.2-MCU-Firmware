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

#include <stdio.h>
#include <string.h>


#include "display.h"
#include "tmp117.h"
#include "circular_buffer.h"
#include "function.h"

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


char myLCDstr[32];
float temperature=10.23;
uint16_t tmpx;

extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);
extern void DDS_Init(void);
extern void DDS_Update(void);
extern void DDS_Calculation(void);
extern void DAC_SendInit(void);
extern void DAC_TEMP_CAL(void);
extern void DAC_Write(uint32_t);
extern void DAC_Write_FAST(void);

extern void Relay_control(uint8_t,uint8_t);
extern void CPLD_control(uint8_t);

extern volatile FunctionalState USB_CDC_End_Line_Received;

extern uint8_t command_buffer[31];

int16_t Enc_Counter = 0;

//Calibration value
float corr_coeff_1;
float corr_coeff_2;
float corr_coeff_3;

CIRC_GBUF_DEF(uint8_t, USB_rx_command_buffer, 30);

uint32_t DAC_tx_buffer;
uint16_t DAC_tx_tmp_buffer[2];
DAC_CONFIG1 cfg;

uint8_t eta_hours,eta_minute,eta_second;

uint8_t CPLD_WORD=0x5;
float DDS_FTW=0;
float DDS_target_frequecny;
float DAC_target_speed;
uint32_t DDS_target_multipiller=1;

float DDS_clock_frequecny=1E7;
float DAC_fullrange_voltage;
float cal_DAC_up_voltage;
float cal_DAC_down_voltage;


uint32_t DAC_code=0x0;
FunctionalState DAC_code_direction;

FunctionalState Need_update_DDS=0;
FunctionalState Ramp_dac_step_complete=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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


  Relay_control(1,0); // x1 mode
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

	send_answer_to_CDC(CLEAR_TYPE_1);

	HAL_TIM_Base_Start_IT(&htim3);

	CPLD_control(CPLD_WORD);


	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

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

          //for(uint16_t tx=0;tx<1000;tx++){
        	  LcdClear_massive();
    		  tmpx=TMP117_get_Temperature(hi2c1);
    		  temperature=tmpx*0.0078125;
    		  sprintf(lcd_buf,"': %.4fmV/s",DAC_target_speed*1000);
              LcdString(1, 1);

//              Enc_Counter = TIM4->CNT;
//        	  sprintf(lcd_buf,"%d",Enc_Counter);
//        	  LcdString(1, 2);

              if(cfg.LDACMODE==1){
            	  sprintf(lcd_buf,"ARM      %01u:%02u:%02u",eta_hours,eta_minute,eta_second);
            	  LcdString(1, 2);
            	  LcdBarLine(DAC_code);	//HAL_Delay(100);
              }
              else
              {
            	  sprintf(lcd_buf,"STANDBY");
            	  LcdString(1, 2);
              }
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
				DAC_SendInit();
				send_answer_to_CDC(DONE_TYPE_1);
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
				DAC_SendInit();
				send_answer_to_CDC(DONE_TYPE_1);
				return;
			}
		}
	}
}

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
		send_answer_to_CDC(ERROR_TYPE_1);
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
		send_answer_to_CDC(ERROR_TYPE_1);
		return;
	}
	// ==== SWEEP command ====
	if(!(strcmp(decoded_string_1,"SWEEP")))
	{
		if(!(strcmp(decoded_string_2,"START"))){
			cmd_SWEEP_START();
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"STOP"))){
				cmd_SWEEP_STOP();
				send_answer_to_CDC(OK_TYPE_2);
				return;
			}
			else
			{
				send_answer_to_CDC(ERROR_TYPE_1);
				return;
			}

		}
	}
	// ==== DAC_SET command ====
	if(!(strcmp(decoded_string_1,"DAC_SET")))
	{
		if(!(strcmp(decoded_string_2,"TOP"))){
			cmd_DAC_SET(DAC_CODE_TOP);
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				cmd_DAC_SET(DAC_CODE_DOWN);
				send_answer_to_CDC(OK_TYPE_2);
				return;
			}
			else
			{
				atof_tmp=atof(decoded_string_2);
				if(atof_tmp>=cal_DAC_down_voltage && atof_tmp<=cal_DAC_up_voltage)
				{
					dac_resolution=(cal_DAC_up_voltage-cal_DAC_down_voltage)/0xFFFFF; // Calculate 1 LSB resolution
					DAC_code=(uint32_t)((atof_tmp-cal_DAC_down_voltage)/dac_resolution);
					cmd_DAC_SET(DAC_code);
					send_answer_to_CDC(OK_TYPE_2);
					return;
				}
				else
				{
					send_answer_to_CDC(ERROR_TYPE_2);
					return;
				}
			}

		}
	}

	// ==== DAC_CAL_TEMP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TEMP")))
	{
		if(!(strcmp(decoded_string_2,"START"))){
			send_answer_to_CDC(RUN_CAL_TYPE_TEMP);
			cmd_CAL(DAC_CAL_TEMP,NONE);
			send_answer_to_CDC(OK_TYPE_1);
			return;
		}
		else
		{
			send_answer_to_CDC(ERROR_TYPE_1);
			return;
		}
	}

	// ==== DAC_CAL_POLY_A command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_A")))
	{
		cmd_CAL(DAC_CAL_POLY_A,atof(decoded_string_2));
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== DAC_CAL_POLY_B command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_B")))
	{
		cmd_CAL(DAC_CAL_POLY_B,atof(decoded_string_2));
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== DAC_CAL_POLY_C command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_C")))
	{
		cmd_CAL(DAC_CAL_POLY_C,atof(decoded_string_2));
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== DAC_CAL_TOP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TOP")))
	{
		if(cmd_CAL(DAC_CAL_TOP,atof(decoded_string_2)))
		{
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}
	}


	// ==== DAC_CAL_DOWN command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_DOWN")))
	{
		if(cmd_CAL(DAC_CAL_DOWN,atof(decoded_string_2)))
		{
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}
	}


	// ==== SWEEP_RATE command ====
	if(!(strcmp(decoded_string_1,"SWEEP_RATE")))
	{
		atof_tmp=atof(decoded_string_2);
		if(atof_tmp<0.001 || atof_tmp>1)
		{
			send_answer_to_CDC(ERROR_TYPE_1);
			return;
		}
		else
		{
			DAC_target_speed=atof_tmp;

			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
	}

	// ==== SWEEP_DIRECTION command ====
	if(!(strcmp(decoded_string_1,"SWEEP_DIRECTION")))
	{
		if(!(strcmp(decoded_string_2,"UP"))){
			DAC_code_direction=1;
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				DAC_code_direction=0;
				send_answer_to_CDC(OK_TYPE_2);
				return;
			}
			else
			{
				send_answer_to_CDC(ERROR_TYPE_1);
				return;
			}

		}
	}

	send_answer_to_CDC(ERROR_TYPE_1);
	return;
}

void Write_to_circ_buffer(uint8_t Buf)
{
	if(CIRC_GBUF_PUSH(USB_rx_command_buffer, &Buf))	CIRC_GBUF_FLUSH(USB_rx_command_buffer); // If out of space, something wrong, clean all !!!
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
