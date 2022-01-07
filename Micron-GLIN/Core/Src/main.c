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
#include "spi.h"
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
extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);

typedef struct DAC_CONFIG1
{
  FunctionalState EN_TMP_CAL;     // Enables and disables the temperature calibration feature
  	  	  	  	  	  	  	  	  //  0 : Temperature calibration feature disabled (default)
  	  	  	  	  	  	  	  	  //  1 : Temperature calibration feature enabled

  uint8_t TNH_MASK;       		  // Mask track and hold (TNH) circuit. This bit is writable only when FSET = 0
  	  	  	  	  	  	  	  	  //  [fast-settling mode] and DIS_TNH = 0 [track-and-hold enabled]
  	  	  	  	  	  	  	  	  //  00: TNH masked for code jump > 2^14 (default)
  	  	  	  	  	  	  	  	  //  01: TNH masked for code jump > 2^15
  	  	  	  	  	  	  	  	  //  10: TNH masked for code jump > 2^13
  	  	  	  	  	  	  	  	  //  11: TNH masked for code jump > 2^12

  FunctionalState LDACMODE;		  // Synchronous or asynchronous mode select bit
  	  	  	  	  	  	  	  	  //  0 : DAC output updated on SYNC rising edge
  	  	  	  	  	  	  	  	  //  1 : DAC updated on LDAC falling edge (default)

  FunctionalState FSDO;		      //  Enable Fast SDO
  	  	  	  	  	  	  	  	  //  0 : Fast SDO disabled (Default)
  	  	  	  	  	  	  	  	  //  1 : Fast SDO enabled

  FunctionalState ENALMP;		  //  Enable ALARM pin to be pulled low, end of temperature calibration cycle
  	  	  	  	  	  	  	  	  //  0 : No alarm on the ALARM pin
  	  	  	  	  	  	  	  	  //  1 : Indicates end of temperature calibration cycle. ALARM pin pulled low.

  FunctionalState DSDO;			  //  Enable SDO (for readback and daisy-chain)
  	  	  	  	  	  	  	  	  //  1 : SDO enabled (default)
  	  	  	  	  	  	  	  	  //  0 : SDO disabled

  FunctionalState FSET;			  //  Fast-settling vs enhanced THD mode
  	  	  	  	  	  	  	  	  //  0 : Fast settling
  	  	  	  	  	  	  	  	  //  1 : Enhanced THD (default)

  uint8_t VREFVAL;    			  // Reference span value bits
  	  	  	  	  	  	  	  	  //  0000: Invalid
	  	  	  	  	  	  	  	  //  0001: Invalid
	  	  	  	  	  	  	  	  //  0010: Reference span = 5 V ± 1.25 V (default)
  	  	  	  	  	  	  	  	  //  0011: Reference span = 7.5 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  0100: Reference span = 10 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  0101: Reference span = 12.5 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  0110: Reference span = 15 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  0111: Reference span = 17.5 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  1000: Reference span = 20 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  1001: Reference span = 22.5 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  1010: Reference span = 25 V ± 1.25 V
  	  	  	  	  	  	  	  	  //  1011: Reference span = 27.5 V± 1.25 V
  	  	  	  	  	  	  	  	  //  1100: Reference span = 30 V ± 1.25 V

  FunctionalState PDN;			  // Powers down and power up the DAC
  	  	  	  	  	  	  	  	  //  0 : DAC power up (default)
  	  	  	  	  	  	  	  	  //  1 : DAC power down
} DAC_CONFIG1;

DAC_CONFIG1 cfg;

//Calibration value
float DDS_clock_frequecny=1E7;
float DAC_fullrange_voltage=19.98133;

float DDS_target_frequecny;
float DAC_target_speed;

uint16_t DDS_tx_buffer[6];
uint32_t DAC_code=0x0;
FunctionalState DAC_code_direction;

uint32_t DAC_tx_buffer;
uint8_t DAC_tx_tmp_buffer[4];

// #define CRICBUF_CLEAN_ON_POP
CIRC_GBUF_DEF(uint8_t, USB_rx_command_buffer, 128);
FunctionalState USB_CDC_End_Line_Received;
uint8_t command_buffer[129];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DDS_Init(void);
void DAC_SendInit(void);
void DAC_Write(uint32_t);
void Write_to_circ_buffer(uint8_t);
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
	cfg.ENALMP=1; // Indicates end of temperature calibration cycle. ALARM pin pulled low
	cfg.FSDO=0; // Fast SDO disabled
	cfg.LDACMODE=1; // DAC updated on LDAC !!!!falling edge!!!!
	cfg.TNH_MASK=0x00; // This bit is writable only when FSET = 0
	cfg.EN_TMP_CAL=1; // Temperature calibration feature enabled
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
  /* USER CODE BEGIN 2 */

  DAC_target_speed=0.001; //  V/s
  DAC_code=0xFFFFF;
  DAC_code_direction=0;

  DDS_target_frequecny=0xFFFFF/(DAC_fullrange_voltage/DAC_target_speed);

  DDS_Init();
  cfg.LDACMODE=0;
  DAC_SendInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(USB_CDC_End_Line_Received)
	  {	  uint8_t i=0;
		  USB_CDC_End_Line_Received=0;
		  while (1) {
			  if (CIRC_GBUF_POP(USB_rx_command_buffer,&command_buffer[i])) command_buffer[i]='\n';
			  if (command_buffer[i]=='\n' || command_buffer[i]=='\r') break;
			  i++;
		  }
		  Parsing_command();
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

/* USER CODE BEGIN 4 */
//==============================================================================================
void DDS_Init(void)
{
  float DDS_FTW=((DDS_target_frequecny*256)/DDS_clock_frequecny)*0xFFFFFFFF;

  DDS_tx_buffer[0]=0xF800; // Enter DAC to Sleep+Reset mode

  DDS_tx_buffer[1]=0x3300; // Write to Frequency 0 Reg, H MSB
  DDS_tx_buffer[1]+=((uint32_t)DDS_FTW >> 24) & 0xFF;

  DDS_tx_buffer[2]=0x2200; // Write to Frequency 0 Reg, L MSBs
  DDS_tx_buffer[2]+=((uint32_t)DDS_FTW >> 16) & 0xFF;

  DDS_tx_buffer[3]=0x3100; // Write to Frequency 0 Reg, H LSBs
  DDS_tx_buffer[3]+=((uint32_t)DDS_FTW >> 8) & 0xFF;

  DDS_tx_buffer[4]=0x2000; // Write to Frequency 0 Reg, L LSBs
  DDS_tx_buffer[4]+=((uint32_t)DDS_FTW & 0xFF);

  DDS_tx_buffer[5]=0xC000; // Exit DAC from Sleep+Reset mode

  HAL_SPI_Transmit(&hspi2,(uint8_t *)DDS_tx_buffer,6,100);

}

//==============================================================================================
void DAC_SendInit(void)
{
	DAC_tx_buffer=0;
	DAC_tx_buffer+=(cfg.PDN & 0x01)<<4;
	DAC_tx_buffer+=(cfg.VREFVAL & 0x0F)<<6;
	DAC_tx_buffer+=(cfg.FSET & 0x01)<<10;
	DAC_tx_buffer+=(cfg.DSDO & 0x01)<<11;
	DAC_tx_buffer+=(cfg.ENALMP & 0x01)<<12;
	DAC_tx_buffer+=(cfg.FSDO & 0x01)<<13;
	DAC_tx_buffer+=(cfg.LDACMODE & 0x01)<<14;
	DAC_tx_buffer+=(cfg.TNH_MASK & 0x03)<<18;
	DAC_tx_buffer+=(cfg.EN_TMP_CAL & 0x01)<<23;
	DAC_tx_buffer+=0x02<<24; // CONFIG1
	DAC_tx_buffer+=0x00<<31; // Write

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);

	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,4,100);
	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_SET);

}

//==============================================================================================
void DAC_Write(uint32_t value)
{
	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t *)DAC_tx_tmp_buffer,4,100);
	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_SET);

	// Speedup hint: Calculate data AFTER send.
	DAC_tx_buffer=0x01000000; // Write DAC-DATA
	DAC_tx_buffer+=(value & 0xFFFFF)<<4;

	DAC_tx_tmp_buffer[0]=(DAC_tx_buffer & 0xFF000000)>>24;
	DAC_tx_tmp_buffer[1]=(DAC_tx_buffer & 0x00FF0000)>>16;
	DAC_tx_tmp_buffer[2]=(DAC_tx_buffer & 0x0000FF00)>>8;
	DAC_tx_tmp_buffer[3]=(DAC_tx_buffer & 0x000000FF);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
//==============================================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_2)
  {
	  if(DAC_code_direction)
	  {
		  if(DAC_code<=0xFFFFF)
		  {
			  DAC_code++;
		  } else  HAL_GPIO_WritePin(GPIOB, COUNT_EN_Pin, GPIO_PIN_SET); // Disable LDAC signal
	  }
	  else
	  {
		  if(DAC_code>=0x0)
		  {
			  DAC_code--;
		  } else  HAL_GPIO_WritePin(GPIOB, COUNT_EN_Pin, GPIO_PIN_SET); // Disable LDAC signal
	  }
	  DAC_Write(DAC_code);
  }

}

//==============================================================================================
void Write_to_circ_buffer(uint8_t Buf)
{
	if(CIRC_GBUF_PUSH(USB_rx_command_buffer, &Buf))	CIRC_GBUF_FLUSH(USB_rx_command_buffer); // If out of space, something wrong, clean all !!!
}

//==============================================================================================
void Parsing_command(void)
{
	char *found;
	char decoded_string_1[129];
	char decoded_string_2[129];
	uint8_t Clear[]="\033c \rEnter command:";
	uint8_t Error[]="\r\n ERROR\n\r\n\r"
			"Usage:\n\r"
			"SWEEP START/STOP to control sweep cycle\n\r"
			"DAC_SET TOP/DOWN to set DAC to 0xFFFFF or 0x0\n\r"
			"\n\r"
			"\n\rEnter command: ";
	uint8_t OK[]="\r\n OK \n\rEnter command: ";

	found = strtok((char *)command_buffer," ");
	if(found!=NULL)
	{
		strcpy(decoded_string_1,found);
	}
	else
	{
		CDC_Transmit_FS(Error, strlen((const char *)Error));  // SEND ERROR TO CDC!!!
	   return;
	}

	found = strtok(NULL,"\r");
	if(found!=NULL)
	{
		strcpy(decoded_string_2,found);
	}
	else
	{
		CDC_Transmit_FS(Error, strlen((const char *)Error));  // SEND ERROR TO CDC!!!
	   return;
	}
// ==== SWEEP command ====
	if(!(strcmp(decoded_string_1,"SWEEP")))
	{
		if(!(strcmp(decoded_string_2,"START"))){
			HAL_GPIO_WritePin(GPIOB, COUNT_EN_Pin, GPIO_PIN_RESET); // Enable LDAC signal
			cfg.LDACMODE=1;
			DAC_SendInit();
			CDC_Transmit_FS(OK, strlen((const char *)OK));
		}
		else
		{
			if(!(strcmp(decoded_string_2,"STOP"))){
						HAL_GPIO_WritePin(GPIOB, COUNT_EN_Pin, GPIO_PIN_SET); // Disable LDAC signal
						cfg.LDACMODE=0;
						DAC_SendInit();
						CDC_Transmit_FS(OK, strlen((const char *)OK));
			}
			else
			{
				CDC_Transmit_FS(Error, strlen((const char *)Error));  // SEND ERROR TO CDC!!!
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
			  DAC_Write(DAC_code);
			  CDC_Transmit_FS(OK, strlen((const char *)OK));
		}
		else
		{
			if(!(strcmp(decoded_string_2,"DOWN"))){
				  DAC_code=0x0;
				  DAC_code_direction=1;
				  cfg.LDACMODE=0;
				  DAC_SendInit();
				  DAC_Write(DAC_code);
				  DAC_Write(DAC_code);
				  CDC_Transmit_FS(OK, strlen((const char *)OK));
			}
			else
			{
				CDC_Transmit_FS(Error, strlen((const char *)Error));  // SEND ERROR TO CDC!!!
				return;
			}
		}
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

