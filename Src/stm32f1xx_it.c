/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include  "LW_Axis_Module.h"
#include  "stm32_define.h"
#include "FQ_S4_Sys.h"

extern LW_Axis_Module_t Axis[Axis_num];

static void USART_IDLE_Handler(UART_HandleTypeDef *huart);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	TIM_HandleTypeDef *htim = &htim2;
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
			LW_Motor_t *sm;
			sm = &(Axis[YSample_Motor].sm);
			if (sm->Step_Count < sm->S_E)
			{
				YSample_Motor_Step_GPIO_Port->BSRR = YSample_Motor_Step_Pin;
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_1, sm->Step_Delay);
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_2, 8);
				if (sm->dir == CCW)
				{
					sm->stepPosition--;
				}
				else
				{
					sm->stepPosition++;
				}
				if (sm->Step_Count < sm->S_S)
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_A)
				{
					sm->Step_Delay = m_GetAccBuf(sm->ID)[sm->Step_Count - sm->S_S];
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_R)
				{
					sm->Step_Delay = sm->Min_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_D)
				{
					sm->Step_Delay = m_GetDecBuf(sm->ID)[sm->Step_Count - sm->S_R];
					sm->Step_Count++;
				}
				else
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
			}
			else
			{
				m_Stop(sm->ID);
				m_CLK_Clr(sm->ID);
				m_Ref_Hold(sm->ID);
				sm->state = STOP;
			}
		}
	}
	/* Capture compare 2 event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
			YSample_Motor_Step_GPIO_Port->BSRR = (uint32_t)YSample_Motor_Step_Pin << 16;
		}
	}
	
if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC3) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
			LW_Motor_t *sm;
			sm = &(Axis[ZSample_Motor].sm);
			if (sm->Step_Count < sm->S_E)
			{
				ZSample_Motor_Step_GPIO_Port->BSRR =ZSample_Motor_Step_Pin;
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_3, sm->Step_Delay);
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_4, 8);
				if (sm->dir == CCW)
				{
					sm->stepPosition--;
				}
				else
				{
					sm->stepPosition++;
				}
				if (sm->Step_Count < sm->S_S)
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_A)
				{
					sm->Step_Delay = m_GetAccBuf(sm->ID)[sm->Step_Count - sm->S_S];
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_R)
				{
					sm->Step_Delay = sm->Min_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_D)
				{
					sm->Step_Delay = m_GetDecBuf(sm->ID)[sm->Step_Count - sm->S_R];
					sm->Step_Count++;
				}
				else
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
			}
			else
			{
				m_Stop(sm->ID);
				m_CLK_Clr(sm->ID);
				m_Ref_Hold(sm->ID);
				sm->state = STOP;
			}
		}
	}
	/* Capture compare 4 event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC4) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
			//htim->Instance->CCMR2;
			ZSample_Motor_Step_GPIO_Port->BSRR = (uint32_t)ZSample_Motor_Step_Pin << 16;
		}
	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	TIM_HandleTypeDef *htim = &htim3;
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
			LW_Motor_t *sm;
			sm = &(Axis[Plug_Motor].sm);
			if (sm->Step_Count < sm->S_E)
			{
				Plug_Motor_Step_GPIO_Port->BSRR = Plug_Motor_Step_Pin;
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_1, sm->Step_Delay);
				SetCompare_Cnt_Delay_16(htim, TIM_CHANNEL_2, 8);
				if (sm->dir == CCW)
				{
					sm->stepPosition--;
				}
				else
				{
					sm->stepPosition++;
				}
				if (sm->Step_Count < sm->S_S)
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_A)
				{
					sm->Step_Delay = m_GetAccBuf(sm->ID)[sm->Step_Count - sm->S_S];
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_R)
				{
					sm->Step_Delay = sm->Min_Delay;
					sm->Step_Count++;
				}
				else if (sm->Step_Count < sm->S_D)
				{
					sm->Step_Delay = m_GetDecBuf(sm->ID)[sm->Step_Count - sm->S_R];
					sm->Step_Count++;
				}
				else
				{
					sm->Step_Delay = sm->SE_Delay;
					sm->Step_Count++;
				}
			}
			else
			{
				m_Stop(sm->ID);
				m_CLK_Clr(sm->ID);
				m_Ref_Hold(sm->ID);
				sm->state = STOP;
			}
		}
	}
	/* Capture compare 2 event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) != RESET)
		{
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
			Plug_Motor_Step_GPIO_Port->BSRR = (uint32_t)Plug_Motor_Step_Pin << 16;
		}
	}
  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	USART_IDLE_Handler(&huart1);
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt.
*/
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static void USART_IDLE_Handler(UART_HandleTypeDef *huart)
{
	uint32_t tmp_flag = 0;

	tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE);

	if ((tmp_flag != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		HAL_DMA_Abort(huart->hdmarx);

		if (huart->State == HAL_UART_STATE_BUSY_TX_RX)
		{
			huart->State = HAL_UART_STATE_BUSY_TX;
		}
		else
		{
			huart->State = HAL_UART_STATE_READY;
		}

		HAL_UART_RxCpltCallback(huart);
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
