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
//float temperature=10.23;
uint16_t tmpx;

extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);
extern void DDS_Init(void);
extern void DDS_Update(void);
extern void DDS_Calculation();
extern void DAC_SendInit(void);
extern void DAC_TEMP_CAL(void);
extern void DAC_Write(uint32_t);
extern void DAC_Write_FAST(void);

extern void Relay_control(uint8_t,uint8_t);
extern void CPLD_control(uint8_t);

extern volatile FunctionalState USB_CDC_End_Line_Received;

extern uint8_t command_buffer[command_buffer_len];

extern float cal_DAC_up_voltage;
extern float cal_DAC_down_voltage;
extern float corr_coeff_1;
extern float corr_coeff_2;
extern float corr_coeff_3;
extern float gain_x2_coeff;
extern float gain_x4_coeff;

extern float C_value[C_value_max_count];
extern float C_leakage[C_value_max_count];

char large_string_buffer[command_buffer_len];

uint8_t eta_hours,eta_minute,eta_second;

int16_t Enc_Counter = 0;
uint32_t Display_timeout=0;

CIRC_GBUF_DEF(uint8_t, USB_rx_command_buffer, command_buffer_len);

uint32_t DAC_tx_buffer;
uint16_t DAC_tx_tmp_buffer[2];
DAC_CONFIG1 cfg;

uint8_t CPLD_WORD;
float DDS_FTW=0;
float DDS_target_frequecny;
float DAC_target_speed;
float amp_target_speed=1E-12;
float ramp_target_speed=0.1;
uint32_t DDS_target_multipiller=1;

uint8_t mode=dU_dt_SCREEN;

float Current_flow=1E-12;
uint8_t C_ref=0;
float Voltage;

float DDS_clock_frequecny=1E7;
float DAC_fullrange_voltage;
float cal_DAC_up_voltage;
float cal_DAC_down_voltage;

uint8_t Current_output_status;

uint32_t DAC_code=0x0;
uint8_t DAC_code_direction;
FunctionalState DAC_code_direction_for_cycle_mode;

FunctionalState Display_need_wakeup=1;
FunctionalState Display_status=0;
FunctionalState Need_update_Display=0;
FunctionalState Need_update_DDS=0;
FunctionalState Ramp_dac_step_complete=0;
FunctionalState CAL_STATE=LOCK_STATE;
FunctionalState Need_off_output=0;
uint8_t Push_start_button=0;
uint8_t Hold_start_button=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void Parsing_USB_command(void);

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

	DAC_target_speed=0.1; //  V/s
	DAC_code=DAC_CODE_MIDDLE;
	DAC_code_direction=DIRECTION_UP_STATE;

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
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	init_LCD();
	/*
  display_screen(Hello_SCREEN);
  LcdUpdate();
  LcdClear_massive();
  HAL_Delay(5000); //WarmUP

  display_screen(Warm_up_SCREEN);
  LcdUpdate();
  LcdClear_massive();
  HAL_Delay(12000); //WarmUP

  display_screen(Ready_SCREEN);
  LcdUpdate();
  LcdClear_massive();
  HAL_Delay(3000); //WarmUP
	 */

	load_data_from_EEPROM();
	TMP117_Initialization(hi2c1);
	DDS_Init();
	DAC_SendInit();
	DAC_Write(DAC_code);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


	output_state(Output_off_STATE);
	CPLD_control(CPLD_OFF_STATE);

	send_answer_to_CDC(CLEAR_TYPE_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if(USB_CDC_End_Line_Received)
		{
			uint8_t i=0,x=0;
			USB_CDC_End_Line_Received=0;
			while (1) {
				if (i>=command_buffer_len) break;
				if (CIRC_GBUF_POP(USB_rx_command_buffer,&command_buffer[i])) command_buffer[i]='\n';
				if (command_buffer[i]=='\n' || command_buffer[i]=='\r') break;
				i++;
			}
			i=0;
			x=0;
			while (command_buffer[i]!='\0' && i<command_buffer_len)
			{
				if (command_buffer[i]=='\177') // Backspace
				{
					if(x>0)x--;
				}
				else
				{
					command_buffer[x]=command_buffer[i];
					x++;
				}
				i++;
			}
			while (x<command_buffer_len)
			{
				command_buffer[x]='\0';
				x++;
			}
			command_buffer[command_buffer_len-1]='\0';

			Parsing_USB_command();
			if(Display_status==0)Display_need_wakeup=1;
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
			//Recalculate_ramp_speed(SWEEP_MODE);

		}

		if(Need_off_output==1)
		{
			Push_start_button=10;
			cmd_SWEEP_STOP();
			output_state(Output_off_STATE);
			Need_off_output=0;
		}

		if(Push_start_button>2 && Push_start_button<10)
		{
			if(mode==VOLT_SCREEN)
			{
				if(Current_output_status==Output_off_STATE)
				{
					cmd_SET_OUTPUT_VOLTAGE(Voltage);
				}
				else
				{
					output_state(Output_off_STATE);
				}
			}
			else
			{
				if(cfg.LDACMODE==0)
				{
					cmd_SWEEP_START();
				}
				else
				{
					cmd_SWEEP_STOP();
				}
			}
			Push_start_button=10;
		}

		if(Need_update_Display && Display_status)
		{
			switch(mode)
			{
			//----------------------------------------------------------//
			case dU_dt_SCREEN:
			{
				display_screen(dU_dt_SCREEN);
			}
			break;
			case AMP_SCREEN:
				display_screen(AMP_SCREEN);
			break;
			case VOLT_SCREEN:
				display_screen(VOLT_SCREEN);
			break;
			}
			LcdUpdate();
			LcdClear_massive();
		}
		if(Enc_Counter!=0)
		{
		switch(mode)
		{
		//----------------------------------------------------------//
		case dU_dt_SCREEN:
		{
			if(Enc_Counter>2 || Enc_Counter<-2)
			{
				Recalculate_ramp_speed(dU_dt_SCREEN, round((ramp_target_speed+Enc_Counter*1E-2)*1E2)/1E2);
			}
			else
			{
				Recalculate_ramp_speed(dU_dt_SCREEN, round((ramp_target_speed+Enc_Counter*1E-3)*1E3)/1E3);
			}
			Enc_Counter=0;
		}
		break;
		case AMP_SCREEN:
/*			if(Enc_Counter>2 || Enc_Counter<-2)
			{
				Recalculate_ramp_speed(AMP_SCREEN, round((amp_target_speed+Enc_Counter*1E-2)*1E2)/1E2);
			}
			else
			{
				Recalculate_ramp_speed(AMP_SCREEN, round((amp_target_speed+Enc_Counter*1E-3)*1E3)/1E3);
			}
			Enc_Counter=0;
*/
		break;
		case VOLT_SCREEN:
			if(Enc_Counter>4 || Enc_Counter<-4)
			{
				cmd_SET_OUTPUT_VOLTAGE(round((Voltage+Enc_Counter*1E-1)*1E1)/1E1);
			}
			else
			{
				cmd_SET_OUTPUT_VOLTAGE(round((Voltage+Enc_Counter*1E-3)*1E3)/1E3);
			}
			Enc_Counter=0;
		break;
		}
		}
		if(Display_need_wakeup)
		{
			Display_need_wakeup=0;
			Poweron_LCD();
			Display_timeout=0;
			Need_update_Display=1;
		} else
		{
			if(Display_status==1) // If display on
			{
				// 1 hour timeout if output is off
				// 1 day timeout if output is on
				if((Display_timeout>72000 && Current_output_status==Output_off_STATE) || (Display_timeout>1728000 && Current_output_status!=Output_off_STATE))
				{
					Poweroff_LCD();
					Display_timeout=0;
				}
			} else Display_timeout=0;
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
	if (htim == &htim3 )// INL correction, each 500ms
	{
		if(cfg.LDACMODE==1)Need_update_DDS=1;
	}

	if (htim == &htim2 )//User interface workload, each 50ms
	{
		Need_update_Display=1;
		Display_timeout++;
		if(Push_start_button!=0)Push_start_button++;
		if(Push_start_button>40)Push_start_button=0;
		Enc_Counter+=((int16_t)TIM4->CNT)/2;
		TIM4->CNT = (uint16_t)(((int16_t)TIM4->CNT) % 2);
		if(!HAL_GPIO_ReadPin(Start_button_GPIO_Port, Start_button_Pin))
		{
			Hold_start_button++;
		}
		else
		{
			Hold_start_button=0;
		}
		if(Hold_start_button>60)
		{
			Need_off_output=1;
		}


	}

}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
//==============================================================================================
__RAM_FUNC void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CPU_IRQ_Pin) // CPU_IRQ signal from Timebase CPLD
	{
		DAC_Write_FAST(); // Сначала стреляем, а потом уже разговариваем
		Ramp_dac_step_complete=1;

		switch(DAC_code_direction)
		{
		//----------------------------------------------------------//
		case DIRECTION_UP_STATE:
			if(DAC_code<=(0xFFFFF-DDS_target_multipiller))
			{
				DAC_code+=DDS_target_multipiller;
			} else  {
				CPLD_control(CPLD_OFF_STATE); // Disable LDAC signal
				//DAC_SendInit();
				//send_answer_to_CDC(DONE_TYPE_1);
				return;
			}
			break;

		case DIRECTION_DOWN_STATE:
			if(DAC_code>=DDS_target_multipiller)
			{
				DAC_code-=DDS_target_multipiller;
			} else {
				CPLD_control(CPLD_OFF_STATE); // Disable LDAC signal
				//DAC_SendInit();
				//send_answer_to_CDC(DONE_TYPE_1);
				return;
			}
			break;

		case DIRECTION_CYCLE_STATE:
			if(DAC_code_direction_for_cycle_mode == 1)
			{
				if(DAC_code<=(0xFFFFF-DDS_target_multipiller))
				{
					DAC_code+=DDS_target_multipiller;
				} else  DAC_code_direction_for_cycle_mode=0;
			} else
			{
				if(DAC_code>=DDS_target_multipiller)
				{
					DAC_code-=DDS_target_multipiller;
				} else DAC_code_direction_for_cycle_mode=1;
			}
			break;
		}
		DAC_tx_buffer=0x01000000; // Write DAC-DATA
		DAC_tx_buffer+=(DAC_code & 0xFFFFF)<<4;
		DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFFFF0000)>>16;
		DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x0000FFFF);
	}

	if((GPIO_Pin == Start_button_Pin) || (GPIO_Pin == Encode_Push_Pin))Display_need_wakeup=1;

	if(GPIO_Pin == Encode_Push_Pin)
	{
		mode++;
		if(mode>VOLT_SCREEN)
			mode=dU_dt_SCREEN;
	}

	if(GPIO_Pin == Start_button_Pin)
		if(Push_start_button==0)Push_start_button=1;
}

void Parsing_USB_command(void)
{
	//char *found;
	char decoded_string_1[command_buffer_len];
	char decoded_string_2[command_buffer_len];
	int num_of_cap;
	float f_value,f_value2;
	uint8_t cdc_counter=0;

	if(sscanf((char *)command_buffer,"%s", decoded_string_1)!=1)
	{
		send_answer_to_CDC(ERROR_TYPE_1);
		return;
	}

	// ==== SWEEP command ====
	if(!(strcmp(decoded_string_1,"SWEEP")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(!(strcmp(decoded_string_2,"START")))
		{
			cmd_SWEEP_START();
			if(mode==VOLT_SCREEN)mode=dU_dt_SCREEN;
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"STOP")))
			{
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
	// ==== CAP_SET command ====
	if(!(strcmp(decoded_string_1,"CAP_SET")))
	{
		if(sscanf((char *)command_buffer,"%s %d", decoded_string_1, &num_of_cap)!=2)
			{
				send_answer_to_CDC(ERROR_TYPE_2);
				return;
			}

			if(cmd_CAP_SET(num_of_cap))
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


	// ==== DAC_SET command ====
	if(!(strcmp(decoded_string_1,"DAC_SET")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

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
				if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
				{
					send_answer_to_CDC(ERROR_TYPE_2);
					return;
				}

				if(cmd_SET_OUTPUT_VOLTAGE(f_value))
				{
					mode=VOLT_SCREEN;
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


	// ==== OUTPUT command ====
	if(!(strcmp(decoded_string_1,"OUTPUT")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(!(strcmp(decoded_string_2,"OFF"))){
			output_state(Output_off_STATE);
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"X1"))){
				output_state(Output_x1_STATE);
				send_answer_to_CDC(OK_TYPE_2);
				return;
			}
			else
			{
				if(!(strcmp(decoded_string_2,"X2"))){
					output_state(Output_x2_STATE);
					send_answer_to_CDC(OK_TYPE_2);
					return;
				}
				else
				{
					if(!(strcmp(decoded_string_2,"X4"))){
						output_state(Output_x4_STATE);
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
	}

	// ==== CAL_STATE command ====
	if(!(strcmp(decoded_string_1,"CAL_STATE")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(!(strcmp(decoded_string_2,"UNLOCK"))){
			CAL_STATE=UNLOCK_STATE;
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"LOCK"))){
				CAL_STATE=LOCK_STATE;
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

	// ==== DAC_CAL_TEMP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TEMP")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

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
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		cmd_CAL(DAC_CAL_POLY_A,f_value);
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== DAC_CAL_POLY_B command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_B")))
	{
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		cmd_CAL(DAC_CAL_POLY_B,f_value);
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== DAC_CAL_POLY_C command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_POLY_C")))
	{
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		cmd_CAL(DAC_CAL_POLY_C,f_value);
		send_answer_to_CDC(OK_TYPE_2);
		return;
	}

	// ==== GAIN_X2_CAL command ====
	if(!(strcmp(decoded_string_1,"GAIN_X2_CAL")))
	{
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(cmd_CAL(GAIN_X2_CAL,f_value))
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

	// ==== GAIN_X4_CAL command ====
	if(!(strcmp(decoded_string_1,"GAIN_X4_CAL")))
	{
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(cmd_CAL(GAIN_X4_CAL,f_value))
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


	// ==== DAC_CAL_TOP command ====
	if(!(strcmp(decoded_string_1,"DAC_CAL_TOP")))
	{
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(cmd_CAL(DAC_CAL_TOP,f_value))
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
		if(sscanf((char *)command_buffer,"%s %f", decoded_string_1, &f_value)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(cmd_CAL(DAC_CAL_DOWN,f_value))
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

	// ==== CAL_C_VALUE command ====
	if(!(strcmp(decoded_string_1,"CAL_C_VALUE")))
	{
		if(sscanf((char *)command_buffer,"%s %d %f %f", decoded_string_1, &num_of_cap, &f_value, &f_value2)!=4)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}
		if(CAL_STATE!=LOCK_STATE)
		{
			write_c_value_to_EEPROM(num_of_cap, f_value, f_value2);
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
		if(sscanf((char *)command_buffer,"%s %s %f", decoded_string_1, decoded_string_2, &f_value)!=3)
		{
			send_answer_to_CDC(ERROR_TYPE_1);
			return;
		}
		if(!(strcmp(decoded_string_2,"AMP")))
		{
			if(Recalculate_ramp_speed(AMP_SCREEN, f_value)==ret_OK)
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
		else
		{
			if(Recalculate_ramp_speed(dU_dt_SCREEN, f_value)==ret_OK)
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
	}

	// ==== SHOW command ====
	if(!(strcmp(decoded_string_1,"SHOW")))
	{

		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(!(strcmp(decoded_string_2,"INFO"))){
			sprintf((char *)large_string_buffer,"\n\rDAC 0xFFFFF voltage calibration constant: %1.6E\n\r",cal_DAC_up_voltage);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"DAC 0x00000 voltage calibration constant: %1.6E\n\r",cal_DAC_down_voltage);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"Linearity correction A: %1.6E\n\r",corr_coeff_1);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"Linearity correction B: %1.6E\n\r",corr_coeff_2);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"Linearity correction C: %1.6E\n\r",corr_coeff_3);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"LT5400 gain X2 correction: %1.6E\n\r",gain_x2_coeff);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"LT5400 gain X4 correction: %1.6E\n\r",gain_x4_coeff);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			for(uint8_t i=0;i<C_value_max_count;i++)
			{
				sprintf((char *)large_string_buffer,"C%d capacitance: %1.6EpF, leakage: %1.6EA/V\n\r",(int)i,C_value[i],C_leakage[i]);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			}
			sprintf((char *)large_string_buffer,"\n\rDAC code: 0x%x\n\r",(unsigned int)DAC_code);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"DDS FTW: 0x%x\n\r",(unsigned int)DDS_FTW);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"CPLD control word: 0x%x\n\r",CPLD_WORD);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"Output mode: 0x%x\n\r",Current_output_status);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			sprintf((char *)large_string_buffer,"Temperature: %2.3f°C\n\r",TMP117_get_Temperature(hi2c1)*0.0078125);while((CDC_Transmit_FS((uint8_t *)large_string_buffer, strlen((const char *)large_string_buffer))!=USBD_OK)&&cdc_counter<0xFF)cdc_counter++;
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"HELP_EXTENDED"))){
				send_answer_to_CDC(EXTENDED_HELP);
				return;
			}
			else
			{
				send_answer_to_CDC(ERROR_TYPE_1);
				return;
			}
		}
	}

	// ==== SWEEP_DIRECTION command ====
	if(!(strcmp(decoded_string_1,"SWEEP_DIRECTION")))
	{
		if(sscanf((char *)command_buffer,"%s %s", decoded_string_1, decoded_string_2)!=2)
		{
			send_answer_to_CDC(ERROR_TYPE_2);
			return;
		}

		if(!(strcmp(decoded_string_2,"UP"))){
			DAC_code_direction=DIRECTION_UP_STATE;
			send_answer_to_CDC(OK_TYPE_2);
			return;
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				DAC_code_direction=DIRECTION_DOWN_STATE;
				send_answer_to_CDC(OK_TYPE_2);
				return;
			}
			else
			{
				if(!(strcmp(decoded_string_2,"CYCLE"))){
					DAC_code_direction=DIRECTION_CYCLE_STATE;
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
