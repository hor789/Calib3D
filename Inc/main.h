/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DebugLED_Pin GPIO_PIN_2
#define DebugLED_GPIO_Port GPIOE
#define Clean_Valve_Pin GPIO_PIN_2
#define Clean_Valve_GPIO_Port GPIOC
#define Clean_In_Pump_Pin GPIO_PIN_0
#define Clean_In_Pump_GPIO_Port GPIOA
#define Clean_Out_Pump_Pin GPIO_PIN_1
#define Clean_Out_Pump_GPIO_Port GPIOA
#define YSample_Motor_Step_Pin GPIO_PIN_7
#define YSample_Motor_Step_GPIO_Port GPIOA
#define YSample_Motor_Dir_Pin GPIO_PIN_4
#define YSample_Motor_Dir_GPIO_Port GPIOC
#define YSample_Motor_Hold_Pin GPIO_PIN_5
#define YSample_Motor_Hold_GPIO_Port GPIOC
#define YSampMotor_Reset_Optic_Pin GPIO_PIN_9
#define YSampMotor_Reset_Optic_GPIO_Port GPIOE
#define ZSampMotor_Reset_Optic_Pin GPIO_PIN_11
#define ZSampMotor_Reset_Optic_GPIO_Port GPIOE
#define Rev1Motor_Reset_Optic_Pin GPIO_PIN_13
#define Rev1Motor_Reset_Optic_GPIO_Port GPIOE
#define Rev2Motor_Reset_Optic_Pin GPIO_PIN_14
#define Rev2Motor_Reset_Optic_GPIO_Port GPIOE
#define ZSample_Motor_Step_Pin GPIO_PIN_13
#define ZSample_Motor_Step_GPIO_Port GPIOB
#define ZSample_Motor_Dir_Pin GPIO_PIN_14
#define ZSample_Motor_Dir_GPIO_Port GPIOB
#define ZSample_Motor_Hold_Pin GPIO_PIN_15
#define ZSample_Motor_Hold_GPIO_Port GPIOB
#define LevelDetect_Pin GPIO_PIN_12
#define LevelDetect_GPIO_Port GPIOD
#define Plug_Motor_Step_Pin GPIO_PIN_15
#define Plug_Motor_Step_GPIO_Port GPIOD
#define Plug_Motor_Dir_Pin GPIO_PIN_6
#define Plug_Motor_Dir_GPIO_Port GPIOC
#define Plug_Motor_Hold_Pin GPIO_PIN_7
#define Plug_Motor_Hold_GPIO_Port GPIOC
#define Led_Power_Ctrl_Pin GPIO_PIN_12
#define Led_Power_Ctrl_GPIO_Port GPIOC
#define Waste_Liq_Full_Pin GPIO_PIN_2
#define Waste_Liq_Full_GPIO_Port GPIOD
#define Clean_liq_Exhaust_Pin GPIO_PIN_4
#define Clean_liq_Exhaust_GPIO_Port GPIOD
#define Mirco_Power_Ctrl_Pin GPIO_PIN_6
#define Mirco_Power_Ctrl_GPIO_Port GPIOD
#define Rev1_End_Optic_Pin GPIO_PIN_5
#define Rev1_End_Optic_GPIO_Port GPIOB
#define Rev2_End_Optic_Pin GPIO_PIN_6
#define Rev2_End_Optic_GPIO_Port GPIOB
#define Rev3_End_Optic_Pin GPIO_PIN_7
#define Rev3_End_Optic_GPIO_Port GPIOB
#define Rev4_End_Optic_Pin GPIO_PIN_8
#define Rev4_End_Optic_GPIO_Port GPIOB
#define DebugBeep_Pin GPIO_PIN_1
#define DebugBeep_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
