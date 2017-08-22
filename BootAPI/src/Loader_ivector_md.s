;******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
;* File Name          : startup_stm32f10x_md.s
;* Author             : MCD Application Team
;* Version            : V3.5.0
;* Date               : 11-March-2011
;* Description        : STM32F10x Medium Density Devices vector table for 
;*                      EWARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Configure the clock system
;*                      - Set the initial PC == __iar_program_start,
;*                      - Set the vector table entries with the exceptions ISR 
;*                        address.
;*                      After Reset the Cortex-M3 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;********************************************************************************
;* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
;* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
;* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
;* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
;* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
;* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************
;
;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        ;;MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec2:CODE:ROOT(2);;CODE:NOROOT(2)
 
        EXTERN  Loader_main
        EXTERN  Boot_NMI_Handler               ; NMI Handler
        EXTERN  Boot_HardFault_Handler         ; Hard Fault Handler
        EXTERN  Boot_MemManage_Handler         ; MPU Fault Handler
        EXTERN  Boot_BusFault_Handler          ; Bus Fault Handler
        EXTERN  Boot_UsageFault_Handler        ; Usage Fault Handler
        EXTERN  Boot_SVC_Handler               ; SVCall Handler
        EXTERN  Boot_DebugMon_Handler          ; Debug Monitor Handler
        EXTERN  Boot_PendSV_Handler            ; PendSV Handler                
        PUBLIC  __vector_table2


        DATA
__vector_table2
        DCD     sfe(CSTACK)
        DCD     Loader_Reset_Handler           ; Reset Handler
        DCD     Boot_NMI_Handler               ; NMI Handler
        DCD     Boot_HardFault_Handler         ; Hard Fault Handler
        DCD     Boot_MemManage_Handler         ; MPU Fault Handler
        DCD     Boot_BusFault_Handler          ; Bus Fault Handler
        DCD     Boot_UsageFault_Handler        ; Usage Fault Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     Boot_SVC_Handler               ; SVCall Handler
        DCD     Boot_DebugMon_Handler          ; Debug Monitor Handler
        DCD     0                         ; Reserved
        DCD     Boot_PendSV_Handler            ; PendSV Handler
        DCD     Loader_SysTick_Handler           ; SysTick Handler

         ; External Interrupts
        DCD     Loader_WWDG_IRQHandler           ; Window Watchdog
        DCD     Loader_PVD_IRQHandler            ; PVD through EXTI Line detect
        DCD     Loader_TAMPER_IRQHandler         ; Tamper
        DCD     Loader_RTC_IRQHandler            ; RTC
        DCD     Loader_FLASH_IRQHandler          ; Flash
        DCD     Loader_RCC_IRQHandler            ; RCC
        DCD     Loader_EXTI0_IRQHandler          ; EXTI Line 0
        DCD     Loader_EXTI1_IRQHandler          ; EXTI Line 1
        DCD     Loader_EXTI2_IRQHandler          ; EXTI Line 2
        DCD     Loader_EXTI3_IRQHandler          ; EXTI Line 3
        DCD     Loader_EXTI4_IRQHandler          ; EXTI Line 4
        DCD     Loader_DMA1_Channel1_IRQHandler  ; DMA1 Channel 1
        DCD     Loader_DMA1_Channel2_IRQHandler  ; DMA1 Channel 2
        DCD     Loader_DMA1_Channel3_IRQHandler  ; DMA1 Channel 3
        DCD     Loader_DMA1_Channel4_IRQHandler  ; DMA1 Channel 4
        DCD     Loader_DMA1_Channel5_IRQHandler  ; DMA1 Channel 5
        DCD     Loader_DMA1_Channel6_IRQHandler  ; DMA1 Channel 6
        DCD     Loader_DMA1_Channel7_IRQHandler  ; DMA1 Channel 7
        DCD     Loader_ADC1_2_IRQHandler         ; ADC1 & ADC2
        DCD     Loader_USB_HP_CAN1_TX_IRQHandler  ; USB High Priority or CAN1 TX
        DCD     Loader_USB_LP_CAN1_RX0_IRQHandler ; USB Low  Priority or CAN1 RX0
        DCD     Loader_CAN1_RX1_IRQHandler       ; CAN1 RX1
        DCD     Loader_CAN1_SCE_IRQHandler       ; CAN1 SCE
        DCD     Loader_EXTI9_5_IRQHandler        ; EXTI Line 9..5
        DCD     Loader_TIM1_BRK_IRQHandler       ; TIM1 Break
        DCD     Loader_TIM1_UP_IRQHandler        ; TIM1 Update
        DCD     Loader_TIM1_TRG_COM_IRQHandler   ; TIM1 Trigger and Commutation
        DCD     Loader_TIM1_CC_IRQHandler        ; TIM1 Capture Compare
        DCD     Loader_TIM2_IRQHandler           ; TIM2
        DCD     Loader_TIM3_IRQHandler           ; TIM3
        DCD     Loader_TIM4_IRQHandler           ; TIM4
        DCD     Loader_I2C1_EV_IRQHandler        ; I2C1 Event
        DCD     Loader_I2C1_ER_IRQHandler        ; I2C1 Error
        DCD     Loader_I2C2_EV_IRQHandler        ; I2C2 Event
        DCD     Loader_I2C2_ER_IRQHandler        ; I2C2 Error
        DCD     Loader_SPI1_IRQHandler           ; SPI1
        DCD     Loader_SPI2_IRQHandler           ; SPI2
        DCD     Loader_USART1_IRQHandler         ; USART1
        DCD     Loader_USART2_IRQHandler         ; USART2
        DCD     Loader_USART3_IRQHandler         ; USART3
        DCD     Loader_EXTI15_10_IRQHandler      ; EXTI Line 15..10
        DCD     Loader_RTCAlarm_IRQHandler       ; RTC Alarm through EXTI Line
        DCD     Loader_USBWakeUp_IRQHandler      ; USB Wakeup from suspend

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Loader_Reset_Handler
        SECTION .text:CODE:REORDER(2)
Loader_Reset_Handler
        LDR     R0, = Loader_main
        BX      R0        
        

        PUBWEAK Loader_SysTick_Handler
        SECTION .text:CODE:REORDER(1)
Loader_SysTick_Handler
        B Loader_SysTick_Handler

        PUBWEAK Loader_WWDG_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_WWDG_IRQHandler
        B Loader_WWDG_IRQHandler

        PUBWEAK Loader_PVD_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_PVD_IRQHandler
        B Loader_PVD_IRQHandler

        PUBWEAK Loader_TAMPER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TAMPER_IRQHandler
        B Loader_TAMPER_IRQHandler

        PUBWEAK Loader_RTC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_RTC_IRQHandler
        B Loader_RTC_IRQHandler

        PUBWEAK Loader_FLASH_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_FLASH_IRQHandler
        B Loader_FLASH_IRQHandler

        PUBWEAK Loader_RCC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_RCC_IRQHandler
        B Loader_RCC_IRQHandler

        PUBWEAK Loader_EXTI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI0_IRQHandler
        B Loader_EXTI0_IRQHandler

        PUBWEAK Loader_EXTI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI1_IRQHandler
        B Loader_EXTI1_IRQHandler

        PUBWEAK Loader_EXTI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI2_IRQHandler
        B Loader_EXTI2_IRQHandler

        PUBWEAK Loader_EXTI3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI3_IRQHandler
        B Loader_EXTI3_IRQHandler

        PUBWEAK Loader_EXTI4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI4_IRQHandler
        B Loader_EXTI4_IRQHandler

        PUBWEAK Loader_DMA1_Channel1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel1_IRQHandler
        B Loader_DMA1_Channel1_IRQHandler

        PUBWEAK Loader_DMA1_Channel2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel2_IRQHandler
        B Loader_DMA1_Channel2_IRQHandler

        PUBWEAK Loader_DMA1_Channel3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel3_IRQHandler
        B Loader_DMA1_Channel3_IRQHandler

        PUBWEAK Loader_DMA1_Channel4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel4_IRQHandler
        B Loader_DMA1_Channel4_IRQHandler

        PUBWEAK Loader_DMA1_Channel5_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel5_IRQHandler
        B Loader_DMA1_Channel5_IRQHandler

        PUBWEAK Loader_DMA1_Channel6_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel6_IRQHandler
        B Loader_DMA1_Channel6_IRQHandler

        PUBWEAK Loader_DMA1_Channel7_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_DMA1_Channel7_IRQHandler
        B Loader_DMA1_Channel7_IRQHandler

        PUBWEAK Loader_ADC1_2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_ADC1_2_IRQHandler
        B Loader_ADC1_2_IRQHandler

        PUBWEAK Loader_USB_HP_CAN1_TX_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USB_HP_CAN1_TX_IRQHandler
        B Loader_USB_HP_CAN1_TX_IRQHandler

        PUBWEAK Loader_USB_LP_CAN1_RX0_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USB_LP_CAN1_RX0_IRQHandler
        B Loader_USB_LP_CAN1_RX0_IRQHandler

        PUBWEAK Loader_CAN1_RX1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_CAN1_RX1_IRQHandler
        B Loader_CAN1_RX1_IRQHandler

        PUBWEAK Loader_CAN1_SCE_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_CAN1_SCE_IRQHandler
        B Loader_CAN1_SCE_IRQHandler

        PUBWEAK Loader_EXTI9_5_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI9_5_IRQHandler
        B Loader_EXTI9_5_IRQHandler

        PUBWEAK Loader_TIM1_BRK_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM1_BRK_IRQHandler
        B Loader_TIM1_BRK_IRQHandler

        PUBWEAK Loader_TIM1_UP_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM1_UP_IRQHandler
        B Loader_TIM1_UP_IRQHandler

        PUBWEAK Loader_TIM1_TRG_COM_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM1_TRG_COM_IRQHandler
        B Loader_TIM1_TRG_COM_IRQHandler

        PUBWEAK Loader_TIM1_CC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM1_CC_IRQHandler
        B Loader_TIM1_CC_IRQHandler

        PUBWEAK Loader_TIM2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM2_IRQHandler
        B Loader_TIM2_IRQHandler

        PUBWEAK Loader_TIM3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM3_IRQHandler
        B Loader_TIM3_IRQHandler

        PUBWEAK Loader_TIM4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_TIM4_IRQHandler
        B Loader_TIM4_IRQHandler

        PUBWEAK Loader_I2C1_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_I2C1_EV_IRQHandler
        B Loader_I2C1_EV_IRQHandler

        PUBWEAK Loader_I2C1_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_I2C1_ER_IRQHandler
        B Loader_I2C1_ER_IRQHandler

        PUBWEAK Loader_I2C2_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_I2C2_EV_IRQHandler
        B Loader_I2C2_EV_IRQHandler

        PUBWEAK Loader_I2C2_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_I2C2_ER_IRQHandler
        B Loader_I2C2_ER_IRQHandler

        PUBWEAK Loader_SPI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_SPI1_IRQHandler
        B Loader_SPI1_IRQHandler

        PUBWEAK Loader_SPI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_SPI2_IRQHandler
        B Loader_SPI2_IRQHandler

        PUBWEAK Loader_USART1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USART1_IRQHandler
        B Loader_USART1_IRQHandler

        PUBWEAK Loader_USART2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USART2_IRQHandler
        B Loader_USART2_IRQHandler

        PUBWEAK Loader_USART3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USART3_IRQHandler
        B Loader_USART3_IRQHandler

        PUBWEAK Loader_EXTI15_10_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_EXTI15_10_IRQHandler
        B Loader_EXTI15_10_IRQHandler

        PUBWEAK Loader_RTCAlarm_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_RTCAlarm_IRQHandler
        B Loader_RTCAlarm_IRQHandler

        PUBWEAK Loader_USBWakeUp_IRQHandler
        SECTION .text:CODE:REORDER(1)
Loader_USBWakeUp_IRQHandler
        B Loader_USBWakeUp_IRQHandler

        END
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
