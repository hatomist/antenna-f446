/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define __text_tim3info_delay 500
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define SD_SPI3_CS_Pin GPIO_PIN_0
#define SD_SPI3_CS_GPIO_Port GPIOC
#define SD_SPI3_MOSI_Pin GPIO_PIN_1
#define SD_SPI3_MOSI_GPIO_Port GPIOC
#define PERIPH_POWER_SWITCH_IT_Pin GPIO_PIN_2
#define PERIPH_POWER_SWITCH_IT_GPIO_Port GPIOC
#define PERIPH_POWER_Pin GPIO_PIN_3
#define PERIPH_POWER_GPIO_Port GPIOC
#define STATUS_UART4_TX_Pin GPIO_PIN_0
#define STATUS_UART4_TX_GPIO_Port GPIOA
#define STATUS_UART4_RX_Pin GPIO_PIN_1
#define STATUS_UART4_RX_GPIO_Port GPIOA
#define STLK_USART_TX_Pin GPIO_PIN_2
#define STLK_USART_TX_GPIO_Port GPIOA
#define STLK_USART_RX_Pin GPIO_PIN_3
#define STLK_USART_RX_GPIO_Port GPIOA
#define PERIPH_BAT_LEVEL_Pin GPIO_PIN_4
#define PERIPH_BAT_LEVEL_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DRV1_PWM_STEP_Pin GPIO_PIN_6
#define DRV1_PWM_STEP_GPIO_Port GPIOA
#define DRV2_PWM_STEP_Pin GPIO_PIN_7
#define DRV2_PWM_STEP_GPIO_Port GPIOA
#define RFD900_USART_RX_Pin GPIO_PIN_5
#define RFD900_USART_RX_GPIO_Port GPIOC
#define RFD900_USART_TX_Pin GPIO_PIN_10
#define RFD900_USART_TX_GPIO_Port GPIOB
#define DRV2_O_DIR_Pin GPIO_PIN_14
#define DRV2_O_DIR_GPIO_Port GPIOB
#define DRV1_O_DIR_Pin GPIO_PIN_15
#define DRV1_O_DIR_GPIO_Port GPIOB
#define GPS_USART_TX_Pin GPIO_PIN_6
#define GPS_USART_TX_GPIO_Port GPIOC
#define GPS_USART_RX_Pin GPIO_PIN_7
#define GPS_USART_RX_GPIO_Port GPIOC
#define DRV2_I_nHOME_Pin GPIO_PIN_8
#define DRV2_I_nHOME_GPIO_Port GPIOC
#define DRV1_I_nHOME_Pin GPIO_PIN_9
#define DRV1_I_nHOME_GPIO_Port GPIOC
#define DRV2_I_nFAULT_Pin GPIO_PIN_8
#define DRV2_I_nFAULT_GPIO_Port GPIOA
#define DRV1_I_nFAULT_Pin GPIO_PIN_9
#define DRV1_I_nFAULT_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SD_SPI3_SCK_Pin GPIO_PIN_10
#define SD_SPI3_SCK_GPIO_Port GPIOC
#define SD_SPI3_MISO_Pin GPIO_PIN_11
#define SD_SPI3_MISO_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MPU9250_I2C_SCL_Pin GPIO_PIN_6
#define MPU9250_I2C_SCL_GPIO_Port GPIOB
#define MPU9250_I2C_SDA_Pin GPIO_PIN_7
#define MPU9250_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
