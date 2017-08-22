/**
  ******************************************************************************
  * @file    EXTI/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.2.0
  * @date    03/01/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include  "Boot_it.h"
#include  "Boot.h"
#include  "stm32f10x.h"


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup EXTI_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void Boot_NMI_Handler(void) 
{
}

//extern void SystemInit (void) ;
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void Boot_HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  if( *((u32 *)0x20001000) == 0x5AA53366 )
      *((u32 *)0x20001000) = 0x98761234 ;
  else
      *((u32 *)0x20001000) = 0x53219876 ;
  // 05fA => Key
  // 04 ==> System Reset
  SCB->AIRCR = 0x05fa0004 ; // Software Reset
  //NVIC_SystemReset() ;
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void Boot_MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void Boot_BusFault_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void Boot_UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void Boot_SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void Boot_DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void Boot_PendSV_Handler(void)
{
}

extern void Boot_SysTick(void) ;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void Boot_SysTick_Handler(void)
{
  //Boot_SysTick() ;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles Window Watchdog interrupt request.
  * @param  None
  * @retval None
  */
void Boot_WWDG_IRQHandler(void)
{
}

/**
  * @brief  This function handles PVD through EXTI Line detect interrupt request.
  * @param  None
  * @retval None
  */
void Boot_PVD_IRQHandler(void)
{
}

/**
  * @brief  This function handles Tamper interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TAMPER_IRQHandler(void)
{
}

/**
  * @brief  This function handles RTC interrupt request.
  * @param  None
  * @retval None
  */
void Boot_RTC_IRQHandler(void)
{
}

/**
  * @brief  This function handles Flash interrupt request.
  * @param  None
  * @retval None
  */
void Boot_FLASH_IRQHandler(void) 
{
}

/**
  * @brief  This function handles RCC interrupt request.
  * @param  None
  * @retval None
  */
void Boot_RCC_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI Line 0 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI0_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI Line 1 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI1_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI Line 2 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI2_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI Line 3 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI3_IRQHandler(void)
{
}

/**
  * @brief  This function handles EXTI Line 4 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI4_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 1 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel1_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 2 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel2_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 3 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel3_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 4 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel4_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 5 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel5_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 6 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel6_IRQHandler(void)
{
}

/**
  * @brief  This function handles DMA1 Channel 7 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_DMA1_Channel7_IRQHandler(void)
{
}

/**
  * @brief  This function handles ADC1 and ADC2 interrupt request.
  * @param  None
  * @retval None
  */
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
void Boot_ADC1_IRQHandler(void)
#else
void Boot_ADC1_2_IRQHandler(void)
#endif
{
}


/**
  * @brief  This function handles EXTI Line 9..5 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI9_5_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM1 Break interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM1_BRK_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM1 Update interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM1_UP_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM1 Trigger and Commutation interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM1_TRG_COM_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM1 Capture Compare interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM1_CC_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM2_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM3_IRQHandler(void)
{
}
          
/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM4_IRQHandler(void)
{
}
          
/**
  * @brief  This function handles I2C1 Event interrupt request.
  * @param  None
  * @retval None
  */
void Boot_I2C1_EV_IRQHandler(void)
{
}

/**
  * @brief  This function handles I2C1 Error interrupt request.
  * @param  None
  * @retval None
  */
void Boot_I2C1_ER_IRQHandler(void)
{
}

/**
  * @brief  This function handles I2C2 Event interrupt request.
  * @param  None
  * @retval None
  */
void Boot_I2C2_EV_IRQHandler(void)
{
}

/**
  * @brief  This function handles I2C1 Error interrupt request.
  * @param  None
  * @retval None
  */
void Boot_I2C2_ER_IRQHandler(void)
{
}
          
/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_SPI1_IRQHandler(void)
{
}

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_SPI2_IRQHandler(void)
{
}
          
/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_USART1_IRQHandler(void)
{
} 

extern void Bootuart_TxRx_Information(void) ;
/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_USART2_IRQHandler(void)
{
  Bootuart_TxRx_Information() ;
  return ;
} 

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_USART3_IRQHandler(void)
{
} 

/**
  * @brief  This function handles EXTI Line 15..10 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_EXTI15_10_IRQHandler(void)
{
}            
    
/**
  * @brief  This function handles RTC alarm through EXTI line interrupt request.
  * @param  None
  * @retval None
  */
void Boot_RTCAlarm_IRQHandler(void)
{
}            

#if defined (STM32F10X_MD) 
/**
  * @brief  This function handles USB Wakeup from suspend line interrupt request.
  * @param  None
  * @retval None
  */
void Boot_USBWakeUp_IRQHandler(void)
{
}    
#else
/**
  * @brief  This function handles HDMI CEC line interrupt request.
  * @param  None
  * @retval None
  */
void Boot_CEC_IRQHandler(void)
{
}

/**
  * @brief  This function handles TIM6 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM6_IRQHandler(void)
{
}

          
/**
  * @brief  This function handles TIM7 interrupt request.
  * @param  None
  * @retval None
  */
void Boot_TIM7_IRQHandler(void)
{
  
}
#endif          

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
