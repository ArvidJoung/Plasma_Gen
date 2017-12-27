/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define BAT_ADC_Pin GPIO_PIN_1
#define BAT_ADC_GPIO_Port GPIOC
#define BATT_THERM_Pin GPIO_PIN_2
#define BATT_THERM_GPIO_Port GPIOC
#define PLA_CTRL_Pin GPIO_PIN_3
#define PLA_CTRL_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOA
#define KEY_PWR_Pin GPIO_PIN_5
#define KEY_PWR_GPIO_Port GPIOC
#define KEY_PWR_EXTI_IRQn EXTI4_15_IRQn
#define PW_CTRL_Pin GPIO_PIN_0
#define PW_CTRL_GPIO_Port GPIOB
#define CHG_STA_Pin GPIO_PIN_1
#define CHG_STA_GPIO_Port GPIOB
#define CHG_CTRL_Pin GPIO_PIN_12
#define CHG_CTRL_GPIO_Port GPIOB
#define USB_ID_Pin GPIO_PIN_15
#define USB_ID_GPIO_Port GPIOB
#define USB_DET_Pin GPIO_PIN_7
#define USB_DET_GPIO_Port GPIOC
#define USB_DET_EXTI_IRQn EXTI4_15_IRQn
#define PULSE_OUT1_Pin GPIO_PIN_8
#define PULSE_OUT1_GPIO_Port GPIOA
#define PULSE_OUT2_Pin GPIO_PIN_11
#define PULSE_OUT2_GPIO_Port GPIOA
#define EN__8V_Pin GPIO_PIN_12
#define EN__8V_GPIO_Port GPIOA
#define STA_LED_B_Pin GPIO_PIN_10
#define STA_LED_B_GPIO_Port GPIOC
#define STA_LED_R_Pin GPIO_PIN_11
#define STA_LED_R_GPIO_Port GPIOC
#define STA_LED_G_Pin GPIO_PIN_12
#define STA_LED_G_GPIO_Port GPIOC
#define OP_LED_Pin GPIO_PIN_8
#define OP_LED_GPIO_Port GPIOB
#define KEY_OP_Pin GPIO_PIN_9
#define KEY_OP_GPIO_Port GPIOB
#define KEY_OP_EXTI_IRQn EXTI4_15_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
 #define USE_FULL_ASSERT    1U 

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

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
