;******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
;* File Name          : startup_stm32f10x_md_vl.s
;* Author             : MCD Application Team
;* Version            : V3.5.0
;* Date               : 11-March-2011
;* Description        : STM32F10x Medium Density Value Line Devices vector table 
;*                      for EWARM toolchain.
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

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit        
        PUBLIC  __vector_table

        DATA
__vector_table
        DCD     sfe(CSTACK)
        DCD     Boot_Reset_Handler             ; Reset Handler
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
        DCD     Boot_SysTick_Handler           ; SysTick Handler

         ; External Interrupts
        DCD     Boot_WWDG_IRQHandler               ; Window Watchdog
        DCD     Boot_PVD_IRQHandler                ; PVD through EXTI Line detect
        DCD     Boot_TAMPER_IRQHandler             ; Tamper
        DCD     Boot_RTC_IRQHandler                ; RTC
        DCD     Boot_FLASH_IRQHandler              ; Flash
        DCD     Boot_RCC_IRQHandler                ; RCC
        DCD     Boot_EXTI0_IRQHandler              ; EXTI Line 0
        DCD     Boot_EXTI1_IRQHandler              ; EXTI Line 1
        DCD     Boot_EXTI2_IRQHandler              ; EXTI Line 2
        DCD     Boot_EXTI3_IRQHandler              ; EXTI Line 3
        DCD     Boot_EXTI4_IRQHandler              ; EXTI Line 4
        DCD     Boot_DMA1_Channel1_IRQHandler      ; DMA1 Channel 1
        DCD     Boot_DMA1_Channel2_IRQHandler      ; DMA1 Channel 2
        DCD     Boot_DMA1_Channel3_IRQHandler      ; DMA1 Channel 3
        DCD     Boot_DMA1_Channel4_IRQHandler      ; DMA1 Channel 4
        DCD     Boot_DMA1_Channel5_IRQHandler      ; DMA1 Channel 5
        DCD     Boot_DMA1_Channel6_IRQHandler      ; DMA1 Channel 6
        DCD     Boot_DMA1_Channel7_IRQHandler      ; DMA1 Channel 7
        DCD     Boot_ADC1_IRQHandler               ; ADC1
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     Boot_EXTI9_5_IRQHandler            ; EXTI Line 9..5
        DCD     Boot_TIM1_BRK_TIM15_IRQHandler     ; TIM1 Break and TIM15
        DCD     Boot_TIM1_UP_TIM16_IRQHandler      ; TIM1 Update and TIM16
        DCD     Boot_TIM1_TRG_COM_TIM17_IRQHandler ; TIM1 Trigger and Commutation and TIM17
        DCD     Boot_TIM1_CC_IRQHandler            ; TIM1 Capture Compare
        DCD     Boot_TIM2_IRQHandler               ; TIM2
        DCD     Boot_TIM3_IRQHandler               ; TIM3
        DCD     Boot_TIM4_IRQHandler               ; TIM4
        DCD     Boot_I2C1_EV_IRQHandler            ; I2C1 Event
        DCD     Boot_I2C1_ER_IRQHandler            ; I2C1 Error
        DCD     Boot_I2C2_EV_IRQHandler            ; I2C2 Event
        DCD     Boot_I2C2_ER_IRQHandler            ; I2C2 Error
        DCD     Boot_SPI1_IRQHandler               ; SPI1
        DCD     Boot_SPI2_IRQHandler               ; SPI2
        DCD     Boot_USART1_IRQHandler             ; USART1
        DCD     Boot_USART2_IRQHandler             ; USART2
        DCD     Boot_USART3_IRQHandler             ; USART3
        DCD     Boot_EXTI15_10_IRQHandler          ; EXTI Line 15..10
        DCD     Boot_RTCAlarm_IRQHandler           ; RTC Alarm through EXTI Line
        DCD     Boot_CEC_IRQHandler                ; HDMI-CEC
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved
        DCD     0                             ; Reserved                        
        DCD     Boot_TIM6_DAC_IRQHandler           ; TIM6 and DAC underrun
        DCD     Boot_TIM7_IRQHandler               ; TIM7                

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Boot_Reset_Handler
        SECTION .text:CODE:REORDER(2)
Boot_Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0
        
        PUBWEAK Boot_NMI_Handler
        SECTION .text:CODE:REORDER(1)
Boot_NMI_Handler
        B Boot_NMI_Handler

        PUBWEAK Boot_HardFault_Handler
        SECTION .text:CODE:REORDER(1)
Boot_HardFault_Handler
        B Boot_HardFault_Handler

        PUBWEAK Boot_MemManage_Handler
        SECTION .text:CODE:REORDER(1)
Boot_MemManage_Handler
        B Boot_MemManage_Handler

        PUBWEAK Boot_BusFault_Handler
        SECTION .text:CODE:REORDER(1)
Boot_BusFault_Handler
        B Boot_BusFault_Handler

        PUBWEAK Boot_UsageFault_Handler
        SECTION .text:CODE:REORDER(1)
Boot_UsageFault_Handler
        B Boot_UsageFault_Handler

        PUBWEAK Boot_SVC_Handler
        SECTION .text:CODE:REORDER(1)
Boot_SVC_Handler
        B Boot_SVC_Handler

        PUBWEAK Boot_DebugMon_Handler
        SECTION .text:CODE:REORDER(1)
Boot_DebugMon_Handler
        B Boot_DebugMon_Handler

        PUBWEAK Boot_PendSV_Handler
        SECTION .text:CODE:REORDER(1)
Boot_PendSV_Handler
        B Boot_PendSV_Handler

        PUBWEAK Boot_SysTick_Handler
        SECTION .text:CODE:REORDER(1)
Boot_SysTick_Handler
        B Boot_SysTick_Handler

        PUBWEAK Boot_WWDG_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_WWDG_IRQHandler
        B Boot_WWDG_IRQHandler

        PUBWEAK Boot_PVD_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_PVD_IRQHandler
        B Boot_PVD_IRQHandler

        PUBWEAK Boot_TAMPER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TAMPER_IRQHandler
        B Boot_TAMPER_IRQHandler

        PUBWEAK Boot_RTC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_RTC_IRQHandler
        B Boot_RTC_IRQHandler

        PUBWEAK Boot_FLASH_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_FLASH_IRQHandler
        B Boot_FLASH_IRQHandler

        PUBWEAK Boot_RCC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_RCC_IRQHandler
        B Boot_RCC_IRQHandler

        PUBWEAK Boot_EXTI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI0_IRQHandler
        B Boot_EXTI0_IRQHandler

        PUBWEAK Boot_EXTI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI1_IRQHandler
        B Boot_EXTI1_IRQHandler

        PUBWEAK Boot_EXTI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI2_IRQHandler
        B Boot_EXTI2_IRQHandler

        PUBWEAK Boot_EXTI3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI3_IRQHandler
        B Boot_EXTI3_IRQHandler

        PUBWEAK Boot_EXTI4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI4_IRQHandler
        B Boot_EXTI4_IRQHandler

        PUBWEAK Boot_DMA1_Channel1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel1_IRQHandler
        B Boot_DMA1_Channel1_IRQHandler

        PUBWEAK Boot_DMA1_Channel2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel2_IRQHandler
        B Boot_DMA1_Channel2_IRQHandler

        PUBWEAK Boot_DMA1_Channel3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel3_IRQHandler
        B Boot_DMA1_Channel3_IRQHandler

        PUBWEAK Boot_DMA1_Channel4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel4_IRQHandler
        B Boot_DMA1_Channel4_IRQHandler

        PUBWEAK Boot_DMA1_Channel5_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel5_IRQHandler
        B Boot_DMA1_Channel5_IRQHandler

        PUBWEAK Boot_DMA1_Channel6_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel6_IRQHandler
        B Boot_DMA1_Channel6_IRQHandler

        PUBWEAK Boot_DMA1_Channel7_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_DMA1_Channel7_IRQHandler
        B Boot_DMA1_Channel7_IRQHandler

        PUBWEAK Boot_ADC1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_ADC1_IRQHandler
        B Boot_ADC1_IRQHandler

        PUBWEAK Boot_EXTI9_5_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI9_5_IRQHandler
        B Boot_EXTI9_5_IRQHandler

        PUBWEAK Boot_TIM1_BRK_TIM15_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM1_BRK_TIM15_IRQHandler
        B Boot_TIM1_BRK_TIM15_IRQHandler

        PUBWEAK Boot_TIM1_UP_TIM16_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM1_UP_TIM16_IRQHandler
        B Boot_TIM1_UP_TIM16_IRQHandler

        PUBWEAK Boot_TIM1_TRG_COM_TIM17_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM1_TRG_COM_TIM17_IRQHandler
        B Boot_TIM1_TRG_COM_TIM17_IRQHandler

        PUBWEAK Boot_TIM1_CC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM1_CC_IRQHandler
        B Boot_TIM1_CC_IRQHandler

        PUBWEAK Boot_TIM2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM2_IRQHandler
        B Boot_TIM2_IRQHandler

        PUBWEAK Boot_TIM3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM3_IRQHandler
        B Boot_TIM3_IRQHandler

        PUBWEAK Boot_TIM4_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM4_IRQHandler
        B Boot_TIM4_IRQHandler

        PUBWEAK Boot_I2C1_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_I2C1_EV_IRQHandler
        B Boot_I2C1_EV_IRQHandler

        PUBWEAK Boot_I2C1_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_I2C1_ER_IRQHandler
        B Boot_I2C1_ER_IRQHandler

        PUBWEAK Boot_I2C2_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_I2C2_EV_IRQHandler
        B Boot_I2C2_EV_IRQHandler

        PUBWEAK Boot_I2C2_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_I2C2_ER_IRQHandler
        B Boot_I2C2_ER_IRQHandler

        PUBWEAK Boot_SPI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_SPI1_IRQHandler
        B Boot_SPI1_IRQHandler

        PUBWEAK Boot_SPI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_SPI2_IRQHandler
        B Boot_SPI2_IRQHandler

        PUBWEAK Boot_USART1_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_USART1_IRQHandler
        B Boot_USART1_IRQHandler

        PUBWEAK Boot_USART2_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_USART2_IRQHandler
        B Boot_USART2_IRQHandler

        PUBWEAK Boot_USART3_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_USART3_IRQHandler
        B Boot_USART3_IRQHandler

        PUBWEAK Boot_EXTI15_10_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_EXTI15_10_IRQHandler
        B Boot_EXTI15_10_IRQHandler

        PUBWEAK Boot_RTCAlarm_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_RTCAlarm_IRQHandler
        B Boot_RTCAlarm_IRQHandler

        PUBWEAK Boot_CEC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_CEC_IRQHandler
        B Boot_CEC_IRQHandler

        PUBWEAK Boot_TIM6_DAC_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM6_DAC_IRQHandler
        B Boot_TIM6_DAC_IRQHandler

        PUBWEAK Boot_TIM7_IRQHandler
        SECTION .text:CODE:REORDER(1)
Boot_TIM7_IRQHandler
        B Boot_TIM7_IRQHandler                

        END
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
