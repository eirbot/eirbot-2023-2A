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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define IA_L_Pin GPIO_PIN_0
#define IA_L_GPIO_Port GPIOC
#define IB_L_Pin GPIO_PIN_1
#define IB_L_GPIO_Port GPIOC
#define IC_L_Pin GPIO_PIN_2
#define IC_L_GPIO_Port GPIOC
#define IA_R_Pin GPIO_PIN_3
#define IA_R_GPIO_Port GPIOC
#define PINCE_Pin GPIO_PIN_0
#define PINCE_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define AL_R_Pin GPIO_PIN_7
#define AL_R_GPIO_Port GPIOA
#define IB_R_Pin GPIO_PIN_4
#define IB_R_GPIO_Port GPIOC
#define IC_R_Pin GPIO_PIN_5
#define IC_R_GPIO_Port GPIOC
#define BL_L_Pin GPIO_PIN_0
#define BL_L_GPIO_Port GPIOB
#define CL_L_Pin GPIO_PIN_1
#define CL_L_GPIO_Port GPIOB
#define H1_L_Pin GPIO_PIN_2
#define H1_L_GPIO_Port GPIOB
#define AL_L_Pin GPIO_PIN_13
#define AL_L_GPIO_Port GPIOB
#define BL_R_Pin GPIO_PIN_14
#define BL_R_GPIO_Port GPIOB
#define CL_R_Pin GPIO_PIN_15
#define CL_R_GPIO_Port GPIOB
#define AH_R_Pin GPIO_PIN_6
#define AH_R_GPIO_Port GPIOC
#define BH_R_Pin GPIO_PIN_7
#define BH_R_GPIO_Port GPIOC
#define CH_R_Pin GPIO_PIN_8
#define CH_R_GPIO_Port GPIOC
#define AH_L_Pin GPIO_PIN_8
#define AH_L_GPIO_Port GPIOA
#define BH_L_Pin GPIO_PIN_9
#define BH_L_GPIO_Port GPIOA
#define CH_L_Pin GPIO_PIN_10
#define CH_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BAT_INT_Pin GPIO_PIN_10
#define BAT_INT_GPIO_Port GPIOC
#define EN_B2_Pin GPIO_PIN_11
#define EN_B2_GPIO_Port GPIOC
#define EN_B3_Pin GPIO_PIN_12
#define EN_B3_GPIO_Port GPIOC
#define PUSHER_Pin GPIO_PIN_2
#define PUSHER_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define NANO_Pin GPIO_PIN_4
#define NANO_GPIO_Port GPIOB
#define H2_L_Pin GPIO_PIN_5
#define H2_L_GPIO_Port GPIOB
#define H3_L_Pin GPIO_PIN_6
#define H3_L_GPIO_Port GPIOB
#define H1_R_Pin GPIO_PIN_7
#define H1_R_GPIO_Port GPIOB
#define H2_R_Pin GPIO_PIN_8
#define H2_R_GPIO_Port GPIOB
#define H3_R_Pin GPIO_PIN_9
#define H3_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
