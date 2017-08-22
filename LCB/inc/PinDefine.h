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
  // PA4 : Analog Input 4, Generator Current 箇痙
  // PA5 : Analog Input 5, Power 12V
  // PA6 : Analog Input 6, Power 15V
  // PA7 : Analog Input 7, Resistance Adj(Resis_adj)
  // PA8 : Output, Status LED(LED22)
  // PA9 : Output, Status LED(LED23)
  // PA10: Output, Status LED(LED24)
  // PA11: Output, External Power Switch(EX_POWER_SW) 场筿方秨闽
  // PA12: Output, VCC Switch(VCC_SW) ERP筿方秨闽

  // PB0 : Input Floating, 场筿方盎代
  // PB1 : NA
  // PB2 : NA
  // PB5 : Output, Battery Output(BAT_POWER) 筿筿方秨闽
  // PB6 : Timer Capture Intput, RPM 
  // PB7 : NA
  // PB8 : NA
  // PB9 : NA
  // PB10: Output, ECB PWM ECB北 
  // PB11: Input Floating, DC Plugin(DC_IN) 
  // PB12: Output,EEPROM DI(93C46_DI)玂痙
  // PB13: Input Floating,EEPROM DO(93C46_DO)玂痙 
  // PB14: Output,EEPROM CLOCK(93C46_SK)玂痙
  // PB15: Output,EEPROM Chip Select(93C46_CS)玂痙

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
  // PC10: Output, Console Power Source Control(CON_P_12V)筿方匡拒(Switch power)
  // PC11: Output, Console Power Source Control(CON_P_BAT)筿方匡拒(Battery)
  // PC12: Output, Battery Connect Relay(BAT_REL)筿筿方秨闽

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
#define   VCC3V3(s)               GPIO_WriteBit(GPIOA,GPIO_Pin_12,s)// ERP筿方秨闽 
#define   ExternalPower(s)        GPIO_WriteBit(GPIOA,GPIO_Pin_11,s)// 场筿方北 
#define   ConsolePower12V(s)      GPIO_WriteBit(GPIOC,GPIO_Pin_10,s)// 祸筿方匡拒(SwithcPower)
#define   ConsolePowerBattery(s)  GPIO_WriteBit(GPIOC,GPIO_Pin_11,s)// 祸筿方匡拒(Battery)
#define   BatteryRelay(s)         GPIO_WriteBit(GPIOC,GPIO_Pin_12,s)// 筿筿方羆秨闽
#define   BatteryForMCU(s)        GPIO_WriteBit(GPIOB,GPIO_Pin_5,s) // 筿筿方秨闽VCC篈
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
#define   STATUSLED1(s)           GPIO_WriteBit(GPIOC,GPIO_Pin_9,s)// LED8 筿ボ縊
#define   STATUSLED2(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_8,s)// LED9 ボ縊
#define   STATUSLED3(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_9,s)// LED10 ゼノ
#define   STATUSLED4(s)           GPIO_WriteBit(GPIOA,GPIO_Pin_10,s)// LED11 计硄癟縊
//------------------------------------------------------------------------------
#define   BatteryChargeIO         TIM3->CCR1
#define   BatteryChargePWM(s)     TIM3->CCR1 = s // 筿筿北 PC6
#define   BatteryChargeOFF        TIM3->CCR1 = 0 
#define   BatteryChargeON         TIM3->CCR1 = _PWM_MAX_ 
#define   BoostIO                 TIM3->CCR2       
#define   BoostPWM(s)             TIM3->CCR2 = s // ど溃筿隔北 PC7
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
// 2014.02.25 絋粄ぃ玂痙 (Pin竲﹟惠絋粄) 
#define   BAT_REVERSE             0
#define   BAT_NORMAL              1
#define   BatteryReverse(s)       (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == s)
//
#define   iWakeupFucb(s)          (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) == s)
#define   iWakeupFRPM(s)          (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == s)
//
#endif /* __PINDEFINE_H__ */


