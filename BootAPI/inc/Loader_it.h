

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOT_IT_H
#define __BOOT_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*
void BootNMI_Handler(void);
void BootHardFault_Handler(void);
void BootMemManage_Handler(void);
void BootBusFault_Handler(void);
void BootUsageFault_Handler(void);
void BootSVC_Handler(void);
void BootDebugMon_Handler(void);
void BootPendSV_Handler(void);
*/
void LoaderSysTick_Handler(void);

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void LoaderWWDG_IRQHandler(void);
void LoaderPVD_IRQHandler(void);
void LoaderTAMPER_IRQHandler(void);
void LoaderRTC_IRQHandler(void);
void LoaderFLASH_IRQHandler(void);
void LoaderRCC_IRQHandler(void);
void LoaderEXTI0_IRQHandler(void);
void LoaderEXTI1_IRQHandler(void);
void LoaderEXTI2_IRQHandler(void);
void LoaderEXTI3_IRQHandler(void);
void LoaderEXTI4_IRQHandler(void);
void LoaderDMA1_Channel1_IRQHandler(void);
void LoaderDMA1_Channel2_IRQHandler(void);
void LoaderDMA1_Channel3_IRQHandler(void);
void LoaderDMA1_Channel4_IRQHandler(void);
void LoaderDMA1_Channel5_IRQHandler(void);
void LoaderDMA1_Channel6_IRQHandler(void);
void LoaderDMA1_Channel7_IRQHandler(void);
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
void LoaderADC1_IRQHandler(void);
#else
void LoaderADC1_2_IRQHandler(void);
#endif
void LoaderCAN1_TX_IRQHandler(void);
void LoaderCAN1_RX0_IRQHandler(void);
void LoaderCAN1_RX1_IRQHandler(void);
void LoaderCAN1_SCE_IRQHandler(void);
void LoaderEXTI9_5_IRQHandler(void);
void LoaderTIM1_BRK_IRQHandler(void);
void LoaderTIM1_UP_IRQHandler(void);
void LoaderTIM1_TRG_COM_IRQHandler(void);
void LoaderTIM1_CC_IRQHandler(void);
void LoaderTIM2_IRQHandler(void);
void LoaderTIM3_IRQHandler(void);
void LoaderTIM4_IRQHandler(void);
void LoaderI2C1_EV_IRQHandler(void);
void LoaderI2C1_ER_IRQHandler(void);
void LoaderI2C2_EV_IRQHandler(void);
void LoaderI2C2_ER_IRQHandler(void);
void LoaderSPI1_IRQHandler(void);
void LoaderSPI2_IRQHandler(void);
void LoaderUSART1_IRQHandler(void);
void LoaderUSART2_IRQHandler(void);
void LoaderUSART3_IRQHandler(void);
void LoaderEXTI15_10_IRQHandler(void);
void LoaderRTCAlarm_IRQHandler(void);
void LoaderOTG_FS_WKUP_IRQHandler(void);
void LoaderTIM5_IRQHandler(void);
void LoaderSPI3_IRQHandler(void);
void LoaderUART4_IRQHandler(void);
void LoaderUART5_IRQHandler(void);
void LoaderTIM6_IRQHandler(void);
void LoaderTIM7_IRQHandler(void);
void LoaderDMA2_Channel1_IRQHandler(void);
void LoaderDMA2_Channel2_IRQHandler(void);
void LoaderDMA2_Channel3_IRQHandler(void);
void LoaderDMA2_Channel4_IRQHandler(void);
void LoaderDMA2_Channel5_IRQHandler(void);
void LoaderETH_IRQHandler(void);
void LoaderETH_WKUP_IRQHandler(void);
void LoaderCAN2_TX_IRQHandler(void);
void LoaderCAN2_RX0_IRQHandler(void);
void LoaderCAN2_RX1_IRQHandler(void);
void LoaderCAN2_SCE_IRQHandler(void);


#endif /* __BOOT_IT_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
