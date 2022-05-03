/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "dac_and_dds_func.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
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

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern uint8_t CDC_Transmit_FS(uint8_t*, uint16_t);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Display_DB6_Pin GPIO_PIN_13
#define Display_DB6_GPIO_Port GPIOC
#define DAC_CLR_Pin GPIO_PIN_0
#define DAC_CLR_GPIO_Port GPIOA
#define Encode_push_Pin GPIO_PIN_3
#define Encode_push_GPIO_Port GPIOA
#define DAC_SYNC_Pin GPIO_PIN_4
#define DAC_SYNC_GPIO_Port GPIOA
#define Control_bus_0_Pin GPIO_PIN_0
#define Control_bus_0_GPIO_Port GPIOB
#define Control_bus_1_Pin GPIO_PIN_1
#define Control_bus_1_GPIO_Port GPIOB
#define Control_bus_2_Pin GPIO_PIN_2
#define Control_bus_2_GPIO_Port GPIOB
#define Control_bus_3_Pin GPIO_PIN_10
#define Control_bus_3_GPIO_Port GPIOB
#define Display_Power_Pin GPIO_PIN_11
#define Display_Power_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define Display_EN_Pin GPIO_PIN_14
#define Display_EN_GPIO_Port GPIOB
#define Display_RW_Pin GPIO_PIN_8
#define Display_RW_GPIO_Port GPIOA
#define Display_RS_Pin GPIO_PIN_9
#define Display_RS_GPIO_Port GPIOA
#define Display_DB4_Pin GPIO_PIN_10
#define Display_DB4_GPIO_Port GPIOA
#define Display_DB5_Pin GPIO_PIN_15
#define Display_DB5_GPIO_Port GPIOA
#define COUNT_EN_Pin GPIO_PIN_3
#define COUNT_EN_GPIO_Port GPIOB
#define Relay_cs_Pin GPIO_PIN_4
#define Relay_cs_GPIO_Port GPIOB
#define Display_DB7_Pin GPIO_PIN_5
#define Display_DB7_GPIO_Port GPIOB
#define Encode_A_Pin GPIO_PIN_6
#define Encode_A_GPIO_Port GPIOB
#define Encode_B_Pin GPIO_PIN_7
#define Encode_B_GPIO_Port GPIOB
#define TMP117_SCL_Pin GPIO_PIN_8
#define TMP117_SCL_GPIO_Port GPIOB
#define TMP117_SDA_Pin GPIO_PIN_9
#define TMP117_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x080827FF
#define DATA_EEPROM_PAGE_SIZE      0x8

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
