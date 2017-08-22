#ifndef __PINDEFINE_H__
#define __PINDEFINE_H__
//------------------------------------------------------------------------------
#include  "stm32f10x_conf.h"

#define   _DefaultGearRate    840// 20130604 829//842

#define   _EnableErrorCode_   1 // 2014.02.18

//#define   _DisableMachineSetError_              1
//#define   _SIM_LCB1_                            1
//#define  DebugMonitor                         1

#ifndef   _SIM_LCB1_
#define   _DisablePWM_CMD_                      1
#endif


//
  // PA0 : ERP WKUP (RPM)
  // PA1 : Output, RS485 DE/RE
  // PA2 : Alternate Function Output, USART2(TX), RS485 TX
  // PA3 : Input Floating, USART2(RX), RS485 RX
  // PA4 : Analog Input 4, Generator Current �w�d
  // PA5 : Analog Input 5, Power 12V
  // PA6 : Analog Input 6, Power 15V
  // PA7 : Analog Input 7, Resistance Adj(Resis_adj)
  // PA8 : Output, Status LED(LED22)
  // PA9 : Output, Status LED(LED23)
  // PA10: Output, Status LED(LED24)
  // PA11: Output, External Power Switch(EX_POWER_SW) �~���q���}��
  // PA12: Output, VCC Switch(VCC_SW) ERP�q���}��

  // PB0 : Input Floating, �~���q������
  // PB1 : NA
  // PB2 : NA
  // PB5 : Output, Battery Output(BAT_POWER) �q���q���}��
  // PB6 : Timer Capture Intput, RPM 
  // PB7 : NA
  // PB8 : NA
  // PB9 : NA
  // PB10: Output, ECB PWM ECB���O���� 
  // PB11: Input Floating, DC Plugin(DC_IN) 
  // PB12: Output,EEPROM DI(93C46_DI)�O�d
  // PB13: Input Floating,EEPROM DO(93C46_DO)�O�d 
  // PB14: Output,EEPROM CLOCK(93C46_SK)�O�d
  // PB15: Output,EEPROM Chip Select(93C46_CS)�O�d

  // PC0 : Analog Input 10, Battery Voltage(BAT_V)
  // PC1 : Analog Input 11, Battery Current(BAT_A)
  // PC2 : Analog Input 12, Boost Current(C1_A)
  // PC3 : Analog Input 13, Generator Current(GE_A)
  // PC4 : Analog Input 14, ECB Current(E_A_ADC)
  // PC5 : Analog Input 15, ECB Voltage(E_V_ADC)
  // PC6 : Alternate Function PWM Output, Battery Charge(Charge)
  // PC7 : Alternate Function PWM Output, Boost Voltage Control(Charge_V)
  // PC8 : NA
  // PC9 : Output, Status LED(LED21)
  // PC10: Output, Console Power Source Control(CON_P_12V)�q�����(Switch power)
  // PC11: Output, Console Power Source Control(CON_P_BAT)�q�����(Battery)
  // PC12: Output, Battery Connect Relay(BAT_REL)�q���q���}��

//
#define   _PWM_FREQ_               20000 // 20Khz
#define   _PWM_MAX_               (SystemCoreClock/_PWM_FREQ_) // 48M / 20k = 2400
#define   _BOOST_MAX_             (unsigned short)(SystemCoreClock/_PWM_FREQ_*0.3) // 720
#define   _CHARGE_PWM_MAX_        (SystemCoreClock/_PWM_FREQ_)
// RS485
#define   RXD                     Bit_RESET
#define   TXD                     Bit_SET
#define   RS485Rx(s)              GPIO_WriteBit(GPIOA,GPIO_Pin_1,s) 
// Power Control
#define   ON                      Bit_RESET
#define   OFF                     Bit_SET
#define   VCC3V3(s)               GPIO_WriteBit(GPIOA,GPIO_Pin_12,s)// ERP�q���}�� 
#define   ExternalPower(s)        GPIO_WriteBit(GPIOA,GPIO_Pin_11,s)// �~���q������ 
#define   ConsolePower12V(s)      GPIO_WriteBit(GPIOC,GPIO_Pin_10,s)// ����q�����(SwithcPower)
#define   ConsolePowerBattery(s)  GPIO_WriteBit(GPIOC,GPIO_Pin_11,s)// ����q�����(Battery)
#define   BatteryRelay(s)         GPIO_WriteBit(GPIOC,GPIO_Pin_12,s)// �q���q���`�}��
#define   BatteryForMCU(s)        GPIO_WriteBit(GPIOB,GPIO_Pin_5,s) // �q���q���}��VCC���A
// EEPROM
#define   HIGH                    Bit_SET
#define   LOW                     Bit_RESET
#define   SK_EErom(s)             GPIO_WriteBit(GPIOB,GPIO_Pin_14,s)
//#define   DO_EErom(s)             GPIO_WriteBit(GPIOB,GPIO_Pin_13,s)
#define   DO_EErom()              GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13)
#define   CS_EErom(s)             GPIO_WriteBit(GPIOB,GPIO_Pin_15,s)
//#define   DI_EErom()              GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)
#define   DI_EErom(s)             GPIO_WriteBit(GPIOB,GPIO_Pin_12,s)

// STATUS LED
#define   STATUSLED1(s)           GPIO_WriteBit(GPIOC,GPIO_Pin_9,s)// LED8 �W�q����ܿO
#define   STATUSLED2(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_8,s)// LED9 ���O���ܿO
#define   STATUSLED3(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_9,s)// LED10 ���Ψ�
#define   STATUSLED4(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_10,s)// LED11 �Ʀ�q�T�O
//------------------------------------------------------------------------------
#define   BatteryChargeIO         TIM3->CCR1
#define   BatteryChargePWM(s)     TIM3->CCR1 = s // �q���R�q���� PC6
#define   BatteryChargeOFF        TIM3->CCR1 = 0 
#define   BatteryChargeON         TIM3->CCR1 = _PWM_MAX_ 
#define   BoostIO                 TIM3->CCR2       
#define   BoostPWM(s)             TIM3->CCR2 = s // �����q������ PC7
#define   BoostOFF                TIM3->CCR2 = 0 
//#define   InclinePWM(s)           TIM3->CCR3 = s
//#define   InclineOFF              TIM3->CCR3 = 0
// 2014.02.25
#define   EcbMaxLimit             (SystemCoreClock/_PWM_FREQ_) // 48M / 20k = 2400
#define   EcbPWM(s)               TIM2->CCR3 = s
#define   EcbOFF                  TIM2->CCR3 = 0
//
#define   iBurnin(s)              (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == s)
//
#define   DCPLUG                  0
#define   DCUNPLUG                1
#define   iDCPlugin(s)            (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == s)
// 2014.02.25 �T�{���O�d (Pin�}�|�ݽT�{) 
#define   BAT_REVERSE             0
#define   BAT_NORMAL              1
#define   BatteryReverse(s)       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == s)
//
#define   iWakeupFucb(s)          (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) == s)
#define   iWakeupFRPM(s)          (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == s)
//
#endif /* __PINDEFINE_H__ */


