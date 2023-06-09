/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ANALOG_BAT_Pin GPIO_PIN_2
#define ANALOG_BAT_GPIO_Port GPIOC
#define READ_BAT_Pin GPIO_PIN_3
#define READ_BAT_GPIO_Port GPIOC
#define ANALOG_SOL_Pin GPIO_PIN_1
#define ANALOG_SOL_GPIO_Port GPIOA
#define READ_SOL_Pin GPIO_PIN_2
#define READ_SOL_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_3
#define CS_GPIO_Port GPIOA
#define INT_C_Pin GPIO_PIN_4
#define INT_C_GPIO_Port GPIOA
#define INT_C_EXTI_IRQn EXTI4_IRQn
#define BT_TX_Pin GPIO_PIN_4
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_5
#define BT_RX_GPIO_Port GPIOC
#define PWR_EN_C_Pin GPIO_PIN_0
#define PWR_EN_C_GPIO_Port GPIOB
#define BTN_DOWN_Pin GPIO_PIN_12
#define BTN_DOWN_GPIO_Port GPIOB
#define BTN_R_Pin GPIO_PIN_13
#define BTN_R_GPIO_Port GPIOB
#define BTN_R_EXTI_IRQn EXTI15_10_IRQn
#define BTN_CNTR_Pin GPIO_PIN_14
#define BTN_CNTR_GPIO_Port GPIOB
#define BTN_CNTR_EXTI_IRQn EXTI15_10_IRQn
#define BTN_L_Pin GPIO_PIN_15
#define BTN_L_GPIO_Port GPIOB
#define BTN_L_EXTI_IRQn EXTI15_10_IRQn
#define BTN_UP_Pin GPIO_PIN_6
#define BTN_UP_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define I2C_RST_C_Pin GPIO_PIN_3
#define I2C_RST_C_GPIO_Port GPIOB
#define LPn_C_Pin GPIO_PIN_5
#define LPn_C_GPIO_Port GPIOB
#define RTC_INT_Pin GPIO_PIN_6
#define RTC_INT_GPIO_Port GPIOB
#define RTC_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define DS3231_I2C_ADDR             0x68

// flash chip defines
//#define CS_Pin GPIO_PIN_6
//#define CS_GPIO_Port GPIOB

// time keeping registers
#define DS3231_TIME_CAL_ADDR        0x00
#define DS3231_ALARM1_ADDR          0x07
#define DS3231_ALARM2_ADDR          0x0B
#define DS3231_CONTROL_ADDR         0x0E
#define DS3231_STATUS_ADDR          0x0F
#define DS3231_AGING_OFFSET_ADDR    0x10
#define DS3231_TEMPERATURE_ADDR     0x11

// control register bits
#define DS3231_CONTROL_A1IE     	0x1		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_A2IE     	0x2		/* Alarm 2 Interrupt Enable */
#define DS3231_CONTROL_INTCN    	0x4		/* Interrupt Control */
#define DS3231_CONTROL_RS1	    	0x8		/* square-wave rate select 2 */
#define DS3231_CONTROL_RS2    		0x10	/* square-wave rate select 2 */
#define DS3231_CONTROL_CONV    		0x20	/* Convert Temperature */
#define DS3231_CONTROL_BBSQW    	0x40	/* Battery-Backed Square-Wave Enable */
#define DS3231_CONTROL_EOSC	    	0x80	/* not Enable Oscillator, 0 equal on */


// status register bits
#define DS3231_STATUS_A1F      		0x01		/* Alarm 1 Flag */
#define DS3231_STATUS_A2F      		0x02		/* Alarm 2 Flag */
#define DS3231_STATUS_BUSY     		0x04		/* device is busy executing TCXO */
#define DS3231_STATUS_EN32KHZ  		0x08		/* Enable 32KHz Output  */
#define DS3231_STATUS_OSF      		0x80		/* Oscillator Stop Flag */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
