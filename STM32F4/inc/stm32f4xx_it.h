/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler();
void HardFault_Handler();
void MemManage_Handler();
void BusFault_Handler();
void UsageFault_Handler();
void SVC_Handler();
void DebugMon_Handler();
void PendSV_Handler();
void SysTick_Handler();
void USART1_IRQHandler();
void USART3_IRQHandler();
void UART4_IRQHandler();
void UART5_IRQHandler();
void EXTI0_IRQHandler();
void I2C1_EV_IRQHandler();
void I2C2_EV_IRQHandler();
void DMA1_Stream1_IRQHandler();
void DMA1_Stream2_IRQHandler();
void DMA1_Stream3_IRQHandler();
void DMA1_Stream4_IRQHandler();
void DMA1_Stream5_IRQHandler();
void DMA1_Stream6_IRQHandler();
void DMA1_Stream7_IRQHandler();
void DMA2_Stream1_IRQHandler();
void DMA2_Stream2_IRQHandler();
void DMA2_Stream3_IRQHandler();
void DMA2_Stream4_IRQHandler();
void DMA2_Stream5_IRQHandler();
void DMA2_Stream6_IRQHandler();
void DMA2_Stream7_IRQHandler();
void EXTI9_5_IRQHandler();
void EXTI2_IRQHandler();
void TIM3_IRQHandler();
void TIM2_IRQHandler();
void FPU_IRQHandler();
#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
