/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FRAM_CS_Pin GPIO_PIN_4
#define FRAM_CS_GPIO_Port GPIOE
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOA
#define BUTTON0_Pin GPIO_PIN_2
#define BUTTON0_GPIO_Port GPIOA
#define BUTTON1_Pin GPIO_PIN_3
#define BUTTON1_GPIO_Port GPIOA
#define ADC2_CM0_Pin GPIO_PIN_6
#define ADC2_CM0_GPIO_Port GPIOA
#define ADC2_VM_Pin GPIO_PIN_4
#define ADC2_VM_GPIO_Port GPIOC
#define ADC1_PHOTO0_Pin GPIO_PIN_5
#define ADC1_PHOTO0_GPIO_Port GPIOC
#define ADC1_PHOTO1_Pin GPIO_PIN_0
#define ADC1_PHOTO1_GPIO_Port GPIOB
#define ADC2_CM1_Pin GPIO_PIN_1
#define ADC2_CM1_GPIO_Port GPIOB
#define DRV_NFAULT_Pin GPIO_PIN_8
#define DRV_NFAULT_GPIO_Port GPIOE
#define PWM1_IN1_Pin GPIO_PIN_9
#define PWM1_IN1_GPIO_Port GPIOE
#define PWM1_IN2_Pin GPIO_PIN_11
#define PWM1_IN2_GPIO_Port GPIOE
#define PWM0_IN1_Pin GPIO_PIN_13
#define PWM0_IN1_GPIO_Port GPIOE
#define PWM0_IN2_Pin GPIO_PIN_14
#define PWM0_IN2_GPIO_Port GPIOE
#define DRV_EN_Pin GPIO_PIN_15
#define DRV_EN_GPIO_Port GPIOE
#define LED5_Pin GPIO_PIN_10
#define LED5_GPIO_Port GPIOB
#define SUCTION_Pin GPIO_PIN_11
#define SUCTION_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define ENC0_B_Pin GPIO_PIN_12
#define ENC0_B_GPIO_Port GPIOD
#define ENC0_A_Pin GPIO_PIN_13
#define ENC0_A_GPIO_Port GPIOD
#define ENC1_B_Pin GPIO_PIN_6
#define ENC1_B_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_7
#define ENC1_A_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED6_Pin GPIO_PIN_0
#define LED6_GPIO_Port GPIOD
#define IR_EN_Pin GPIO_PIN_1
#define IR_EN_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_6
#define VCP_TX_GPIO_Port GPIOB
#define VCP_RX_Pin GPIO_PIN_7
#define VCP_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
