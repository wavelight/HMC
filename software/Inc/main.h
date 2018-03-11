/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define USED_Pin GPIO_PIN_3
#define USED_GPIO_Port GPIOE
#define USEDC0_Pin GPIO_PIN_0
#define USEDC0_GPIO_Port GPIOC
#define ANALOG_VIN_Pin GPIO_PIN_1
#define ANALOG_VIN_GPIO_Port GPIOC
#define USEDC3_Pin GPIO_PIN_3
#define USEDC3_GPIO_Port GPIOC
#define USEDA0_Pin GPIO_PIN_0
#define USEDA0_GPIO_Port GPIOA
#define ANALOG_VSENS_1_Pin GPIO_PIN_1
#define ANALOG_VSENS_1_GPIO_Port GPIOA
#define ANALOG_VSENS_2_Pin GPIO_PIN_2
#define ANALOG_VSENS_2_GPIO_Port GPIOA
#define ANALOG_VSENS_3_Pin GPIO_PIN_3
#define ANALOG_VSENS_3_GPIO_Port GPIOA
#define USEDA4_Pin GPIO_PIN_4
#define USEDA4_GPIO_Port GPIOA
#define USEDA5_Pin GPIO_PIN_5
#define USEDA5_GPIO_Port GPIOA
#define USEDA6_Pin GPIO_PIN_6
#define USEDA6_GPIO_Port GPIOA
#define USEDA7_Pin GPIO_PIN_7
#define USEDA7_GPIO_Port GPIOA
#define USEDC4_Pin GPIO_PIN_4
#define USEDC4_GPIO_Port GPIOC
#define ANALOG_ISENS_1_Pin GPIO_PIN_5
#define ANALOG_ISENS_1_GPIO_Port GPIOC
#define ANALOG_ISENS_2_Pin GPIO_PIN_0
#define ANALOG_ISENS_2_GPIO_Port GPIOB
#define ANALOG_ISENS_3_Pin GPIO_PIN_1
#define ANALOG_ISENS_3_GPIO_Port GPIOB
#define USEDB2_Pin GPIO_PIN_2
#define USEDB2_GPIO_Port GPIOB
#define USEDB10_Pin GPIO_PIN_10
#define USEDB10_GPIO_Port GPIOB
#define DRV_CS_Pin GPIO_PIN_12
#define DRV_CS_GPIO_Port GPIOB
#define DRV_SCK_Pin GPIO_PIN_13
#define DRV_SCK_GPIO_Port GPIOB
#define DRV_MISO_Pin GPIO_PIN_14
#define DRV_MISO_GPIO_Port GPIOB
#define DRV_MOSI_Pin GPIO_PIN_15
#define DRV_MOSI_GPIO_Port GPIOB
#define DRV_CAL_Pin GPIO_PIN_8
#define DRV_CAL_GPIO_Port GPIOD
#define DRV_FAULT_Pin GPIO_PIN_9
#define DRV_FAULT_GPIO_Port GPIOD
#define DRV_ENABLE_Pin GPIO_PIN_10
#define DRV_ENABLE_GPIO_Port GPIOD
#define DRV_LED_Pin GPIO_PIN_11
#define DRV_LED_GPIO_Port GPIOD
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define HALL_A_Pin GPIO_PIN_6
#define HALL_A_GPIO_Port GPIOC
#define HALL_B_Pin GPIO_PIN_7
#define HALL_B_GPIO_Port GPIOC
#define HALL_C_Pin GPIO_PIN_8
#define HALL_C_GPIO_Port GPIOC
#define USEDA9_Pin GPIO_PIN_9
#define USEDA9_GPIO_Port GPIOA
#define USEDC10_Pin GPIO_PIN_10
#define USEDC10_GPIO_Port GPIOC
#define USEDC12_Pin GPIO_PIN_12
#define USEDC12_GPIO_Port GPIOC
#define USEDD4_Pin GPIO_PIN_4
#define USEDD4_GPIO_Port GPIOD
#define USEDD5_Pin GPIO_PIN_5
#define USEDD5_GPIO_Port GPIOD
#define USEDB6_Pin GPIO_PIN_6
#define USEDB6_GPIO_Port GPIOB
#define USEDB9_Pin GPIO_PIN_9
#define USEDB9_GPIO_Port GPIOB
#define USEDE0_Pin GPIO_PIN_0
#define USEDE0_GPIO_Port GPIOE
#define USEDE1_Pin GPIO_PIN_1
#define USEDE1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
