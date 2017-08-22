/**
  ******************************************************************************
  * @file    Boot.c
  * @author  JHT Team
  * @version V1.0.0
  * @date    03/12/2010
  * @brief   
  ******************************************************************************
*/ 

//------------------------------------------------------------------------------
#include  "BOOTflash.h"
#include  "Boot.h"
#include  "UCBInfo.h"
#include  "stm32f10x.h"
#include  "Bootuart.h"
#include  "Loader.h"

//------------------------------------------------------------------------------
void Boot_main(void) ;

#define   LOADER_StartAddr               ((uint32_t)0x08003C00)
#define   LOADER_EndAddr                 ((uint32_t)0x08005FFF)


#define   _NO_USER_API_               0x5AA53366
#define   _SOFTWARE_ERR_              0x53219876
#define   _LOADER_ERR_                0x98761234
__no_init unsigned long LoaderIntoMode @ 0x20001000 ;
__root void * const BootMainAddress              @ ".bootaddress" = (void *)Boot_main ;

__no_init unsigned long BootLEDTime ;
void Boot_LED(unsigned char Data) ;
void Boot_SysTick(void) ;



//------------------------------------------------------------------------------
char Boot_CheckUserInformation(const unsigned char *Src,const unsigned char *Dist) 
{
  while( *Src != 0 )
      {
      if( *Src != *Dist )
          {
          return _DataNotMatch_ ;
          }
      Src += 1 ;
      Dist += 1 ;
      } 
  return _DataMatch_;
}


//------------------------------------------------------------------------------
char Boot_CheckUserProgram(void)
{
  unsigned long *pulApp;
  char ret_status ;
  ret_status = _UserAPIError_ ;
  //----------------------------------------------------------------------------
  if( Boot_CheckUserInformation(&Boot_Manufacture[0],(const unsigned char *)(0x08000000+_UInfo_ManufactureIndex_)) == _DataMatch_ )
      if( Boot_CheckUserInformation(&Boot_McuType[0],(const unsigned char *)(0x08000000+_UInfo_McuTypeIndex_)) == _DataMatch_ )
          if( Boot_CheckUserInformation(&Boot_ModuleName[0],(const unsigned char *)(0x08000000+_UInfo_ModuleNameIndex_)) == _DataMatch_ )
              if( Boot_CheckUserInformation(&Boot_ModuleNo[0],(const unsigned char *)(0x08000000+_UInfo_ModuleNoIndex_)) == _DataMatch_ ) 
                  if( Boot_CheckUserInformation(&Boot_Product[0],(const unsigned char *)(0x08000000+_UInfo_ProductIndex_)) == _DataMatch_ )                            
                      ret_status = _UserAPIOk_ ;
 
  //----------------------------------------------------------------------------
  // Check User Main Function Address is OK
  // See if the first location is 0xfffffffff or something that does not
  // look like a stack pointer, or if the second location is 0xffffffff or
  // something that does not look like a reset vector.
  //
  if(  ret_status == _UserAPIOk_ )
      {
      pulApp = (unsigned long *)APP_START_ADDRESS;
      if((pulApp[0] == 0xffffffff) || ((pulApp[0] & 0xfff00000) != 0x20000000) ||
          (pulApp[1] == 0xffffffff) || ((pulApp[1] & 0xfffff000) < 0x08006000) )  
          {
          //
          // App starting stack pointer or PC is not valid, so force an update.
          //
          ret_status = _UserAPIError_ ;
          }
      }
  //----------------------------------------------------------------------------
  return ret_status ;
}




//------------------------------------------------------------------------------
char Boot_CheckLoaderProgram(void)
{
  unsigned long *pulApp;
  char ret_status ;
  ret_status = _LoaderAPIError_ ;
  //----------------------------------------------------------------------------
  if( Boot_CheckUserInformation(&Boot_Manufacture[0],(const unsigned char *)(0x08000000+_LInfo_ManufactureIndex_)) == _DataMatch_ )
      if( Boot_CheckUserInformation(&Boot_McuType[0],(const unsigned char *)(0x08000000+_LInfo_McuTypeIndex_)) == _DataMatch_ )
          if( Boot_CheckUserInformation(&Boot_ModuleName[0],(const unsigned char *)(0x08000000+_LInfo_ModuleNameIndex_)) == _DataMatch_ )
              //if( Boot_CheckUserInformation(&Boot_ModuleNo[0],(const unsigned char *)(0x08000000+_LInfo_ModuleNoIndex_)) == _DataMatch_ ) 
                  //if( Boot_CheckUserInformation(&Boot_Product[0],(const unsigned char *)(0x08000000+_LInfo_ProductIndex_)) == _DataMatch_ )                            
                      ret_status = _LoaderAPIOk_ ;
 
  pulApp =(unsigned long *)(0x08000000 + _LInfo_LoaderStartAddress_) ;
  if( *pulApp == 0xFFFFFFFF || *pulApp == 0 || ((*pulApp & 0xfffffC00) < 0x08003C00) )
      ret_status = _LoaderAPIError_ ;
  
  //----------------------------------------------------------------------------
  // Check User Main Function Address is OK
  // See if the first location is 0xfffffffff or something that does not
  // look like a stack pointer, or if the second location is 0xffffffff or
  // something that does not look like a reset vector.
  //
  if(  ret_status == _LoaderAPIOk_ )
      {
      pulApp = (unsigned long *)LOADER_START_ADDRESS;
      if((pulApp[0] == 0xffffffff) || ((pulApp[0] & 0xfff00000) != 0x20000000) ||
          (pulApp[1] == 0xffffffff) || ((pulApp[1] & 0xfffffC00) < 0x08003C00) )  
          {
          //
          // App starting stack pointer or PC is not valid, so force an update.
          //
          ret_status = _LoaderAPIError_ ;
          }
      }
  //----------------------------------------------------------------------------
  return ret_status ;
}




//------------------------------------------------------------------------------
void Boot_CallApplication(unsigned long ulStartAddr)
{
  //
  // Set the vector table to the beginning of the app in flash.
  //
  SCB->VTOR = ulStartAddr ;
  //
  // Load the stack pointer from the application's vector table.
  //
  __asm("    ldr     r1, [r0]\n"
        "    mov     sp, r1");

  //
  // Load the initial PC from the application's vector table and branch to
  // the application's entry point.
  //
  __asm("    ldr     r0, [r0, #4]\n"
        "    bx      r0\n");
  
  return ;
}




//------------------------------------------------------------------------------
void Boot_DisableIRQn(char All)
{
  SCB->SHCSR &= ~SCB_SHCSR_SYSTICKACT ;
  SysTick->VAL   = 0 ;                      /* Load the SysTick Counter Value */
  SysTick->CTRL  = 0 ;
  if( All != 0 )
      {
      NVIC->ICER[0] = 0xFFFFFFFF ;
      NVIC->ICER[1] = 0xFFFFFFFF ;
      NVIC->ICER[2] = 0xFFFFFFFF ;
      }
  else
      {
      NVIC->ICER[0] = 0xFFFFFFFF ; //  0~31
      NVIC->ICER[1] = 0xFFFFFFFF ; // 32~63
      NVIC->ICER[2] = 0xFFFFFFF7 ; // 64~67 //OTG = 67
      }
  
  return ;
}



#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void Boot_NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup)
{
  /* Check the parameters */
  assert_param(IS_NVIC_PRIORITY_GROUP(NVIC_PriorityGroup));
  
  /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup;
}


void Boot_NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{ 
  /* Check the parameters */
  assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
  assert_param(IS_NVIC_OFFSET(Offset));  
   
  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}


void Boot_HardwareInitial(void)
{
  //============================================================================
  // Hardware Clock Initial
  // RCC GPIOA,GPIOB,GPIOC,GPIOD,USART2,USART1 
  RCC->APB2ENR |= 0x0000403D ; // Enable GPIOA,GPIOB,GPIOC,GPIOD,USART1 Clock
  RCC->APB1ENR |= 0x00020000 ; // Enable USART2 Clock
  // Set GPIO
  // CNF[1:0]       MODE[1:0]
  //     |          00: Input mode (reset state)
  //     |          01: Output mode, max speed 10 MHz.
  //     |          10: Output mode, max speed 2 MHz.
  //     |          11: Output mode, max speed 50 MHz.
  //     v
  // In input mode (MODE[1:0]=00):
  // 00: Analog mode
  // 01: Floating input (reset state)
  // 10: Input with pull-up / pull-down
  // 11: Reserved
  // In output mode (MODE[1:0] > 00):
  // 00: General purpose output push-pull
  // 01: General purpose output Open-drain
  // 10: Alternate function output Push-pull
  // 11: Alternate function output Open-drain
  //----------------------------------------------------------------------------
  // PA0 : 
  // PA1 : Output, RS485 DE/RE
  // PA2 : Alternate Function Output, USART2(TX), RS485 TX
  // PA3 : Input Floating, USART2(RX), RS485 RX
  // PA4 : Analog Input 4, Incline Position(INC_POS)
  // PA5 : Analog Input 5, Power 12V
  // PA6 : Analog Input 6, Power 15V
  // PA7 : Analog Input 7, Resistance Adj(Resis_adj)
  // PA8 : Timer Capture Intput, RPM
  // PA9 : Alternate Function Output, USART1(TX), RS232 TX
  // PA10: Input Floating, USART1(RX), RS232 RX
  // PA11: Output, External Power Switch(EX_POWER_SW)
  // PA12: Output, VCC Switch(VCC_SW)
  // 
  //          Pin  76543210
  //GPIOA->CRL &= ~0xffffffff ;
  //GPIOA->CRL |=  0x00004934 ;
  //          Pin  FEDCBA98    
  //GPIOA->CRH &= ~0x000fffff ;
  //GPIOA->CRH |=  0x000334b4 ; // Input Floating
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0010 0001 1000 0000 0000
  //GPIOA->BSRR =  0x00021800 ; 
  //
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
  //          Pin  76543210
  GPIOA->CRL &= ~0xffffffff ;
  GPIOA->CRL |=  0x00004933 ;
  //          Pin  FEDCBA98    
  GPIOA->CRH &= ~0x000fffff ;
  GPIOA->CRH |=  0x00033333 ; 
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0010 0001 1000 0000 0000
  GPIOA->BSRR =  0x00001f00 ; 
  
  
  //
  // PB0 : Input Floating, DC Plugin(DC_IN)
  // PB1 : Input Floating, Battery Reverse Check (BAT_VHK)
  // PB5 : Output, Battery Output(BAT_POWER)
  // PB6 : Timer Capture Intput, RPM //Output, Status LED(LED21)
  // PB7 : Output, Status LED(LED22)
  // PB8 : Output, Status LED(LED23)
  // PB9 : Output, Status LED(LED24)
  // PB10: Output, EEPROM DI(93C46_DI)
  // PB11: Input Floating, EEPROM DO(93C46_DO)
  // PB12: Output, EEPROM CLOCK(93C46_SK)
  // PB13: Output, EEPROM Chip Select(93C46_CS)
  // PB14: Input Floating, Burn-in
  // PB15: Output, Status LED(LED21)
  //          Pin  76543210
  //GPIOB->CRL &= ~0xfff000ff ;
  //GPIOB->CRL |=  0x34300044 ; // 0x33300044
  //          Pin  FEDCBA98    
  //GPIOB->CRH &= ~0xffffffff ; // 0x0fffffff
  //GPIOB->CRH |=  0x34334333 ; // 0x04334333
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0000 1011 0111 1110 0000
  //GPIOB->BSRR =  0x0000b7e0 ; // 0x000037e0 
  //
  
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
  //          Pin  76543210
  GPIOB->CRL &= ~0x0ff0000f ;
  GPIOB->CRL |=  0x04300004 ; // 0x33300044
  //          Pin  FEDCBA98    
  GPIOB->CRH &= ~0xffffff00 ; // 0x0fffffff
  GPIOB->CRH |=  0x33434300 ; // 0x04334333
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0000 1101 0100 0010 0000
  GPIOB->BSRR =  0x0000d420 ; // 0x000037e0 
  //
  
  // PC0 : Analog Input 10, Battery Voltage(BAT_V)
  // PC1 : Analog Input 11, Battery Current(BAT_A)
  // PC2 : Analog Input 12, Boost Current(C1_A)
  // PC3 : Analog Input 13, Incline Current(INC_A)
  // PC4 : Analog Input 14, Console Current(UCB_A)
  // PC6 : Alternate Function PWM Output, Battery Charge(Charge)
  // PC7 : Alternate Function PWM Output, Boost Voltage Control(Charge_V)
  // PC8 : Alternate Function PWM Output, Incline Control(INC_PWM)
  // PC10: Output, Console Power Source Control(CON_P_12V)
  // PC11: Output, Console Power Source Control(CON_P_BAT)
  // PC12: Output, Battery Connect Relay(BAT_REL)
  //
  //          Pin  76543210
  //GPIOC->CRL &= ~0xff0fffff ;
  //GPIOC->CRL |=  0x33044444 ;
  //          Pin  FEDCBA98    
  //GPIOC->CRH &= ~0x000fff0f ;
  //GPIOC->CRH |=  0x00033303 ; 
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0000 0001 1101 1100 0000
  //GPIOC->BSRR =  0x00001dc0 ; 
  //
  
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
  //          Pin  76543210
  GPIOC->CRL &= ~0xffffffff ;
  GPIOC->CRL |=  0x33444444 ;
  //          Pin  FEDCBA98    
  GPIOC->CRH &= ~0x000ffff0 ;
  GPIOC->CRH |=  0x00033330 ; 
  // FEDC BA98 7654 3210 FEDC BA98 7654 3210
  // 0000 0000 0000 0000 0001 1110 1100 0000
  GPIOC->BSRR =  0x00001ec0 ; 
  //
  
  
  // PD2 : Output, External Poert for Battery Charge
  //          Pin  76543210
  //GPIOC->CRL &= ~0x00000f00 ;
  //GPIOC->CRL |=  0x00000300 ; 
  //GPIOC->BSRR =  0x00000040 ; // Default High Level
  //============================================================================
  return ;
}


//##############################################################################
//
//##############################################################################
void Boot_main(void)
{
  //unsigned int _LEDTime = 0;
  
  //
  LoaderIntoMode = _NO_USER_API_ ;
  Boot_DisableIRQn(1) ;
  Boot_NVIC_SetVectorTable(NVIC_VectTab_FLASH,0) ;
  /* Configure one bit for preemption priority */
  Boot_NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //
  Boot_HardwareInitial() ;
  Bootuart_Initial() ;
  // Open Power Switch
  /*
  VCC3V3(s)               GPIO_WriteBit(GPIOA,GPIO_Pin_12,s) 
  ExternalPower(s)        GPIO_WriteBit(GPIOA,GPIO_Pin_11,s) 
  ConsolePower12V(s)      GPIO_WriteBit(GPIOC,GPIO_Pin_10,s)
  ConsolePowerBattery(s)  GPIO_WriteBit(GPIOC,GPIO_Pin_11,s)
  BatteryRelay(s)         GPIO_WriteBit(GPIOC,GPIO_Pin_12,s)
  BatteryForMCU(s)        GPIO_WriteBit(GPIOB,GPIO_Pin_5,s) 
  iDCPlugin(s)            (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == s)
  */
  GPIOA->BRR = GPIO_Pin_12|GPIO_Pin_11 ; // VCC3.3
  GPIOC->BRR = GPIO_Pin_10 ; // Power ON
  //
  /* Mask by Kunlung 20130523
  // System Stick 1ms
  SysTick->LOAD  = ((SystemCoreClock / 1000) & SysTick_LOAD_RELOAD_Msk) - 1;      // set reload register 
  SCB->SHP[((uint32_t)(SysTick_IRQn) & 0xF)-4] = ((((1<<__NVIC_PRIO_BITS) - 1) << (8 - __NVIC_PRIO_BITS)) & 0xff);  // set Priority for Cortex-M3 System Interrupts
  SysTick->VAL   = 0;                                           // Load the SysTick Counter Value 
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
                   SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                     // Enable SysTick IRQ and SysTick Timer 
                                                                // Function successful
  */
  //
  for(;;)
      {
      Bootuart_RxProcess() ;
      //------------------------------------------------------------------------
      // Add by Kunlung 20130523
      BootLEDTime += 1 ;//BootLEDTime
      if( BootLEDTime >= 100000 )//BootLEDTime
          {
          if( BootLEDTime >= 200000 )//BootLEDTime
              {
              BootLEDTime = 0 ;//BootLEDTime
              Boot_LED(0) ;
              }
          else
              if( BootLEDTime == 100000 )//BootLEDTime
                  Boot_LED(1) ;
          }
      //
      }
}

//##############################################################################
void Boot_LED(unsigned char Data)
{
  if( Data == 0 )
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0011 1100 0000
      // 0000 0000 0000 0000 1000 0001 1000 0000 
      //GPIOB->BRR =  0x00008180 ; // LCBA
      // 0000 0000 0000 0000 0000 0111 0000 0000 
      //GPIOA->BRR =  0x00000700 ; // 1x LCB
      GPIOA->BRR = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//|GPIO_Pin_12|GPIO_Pin_11;  
      GPIOC->BRR =  GPIO_Pin_9 ; // 1x LCB
      }
  else
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0011 1100 0000
      // 0000 0000 0000 0000 1000 0001 1000 0000
      //GPIOB->BSRR =  0x00008180 ;// LCBA
      /// 0000 0000 0000 0000 0000 0111 0000 0000 
      //GPIOA->BSRR =  0x00001F00 ;// 1x LCB
      GPIOA->BSRR = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
      GPIOC->BSRR =  GPIO_Pin_9 ;// 1x LCB  
      }
  return ;
}


//##############################################################################
void Boot_SysTick(void)
{
  /*
  BootLEDTime += 1 ;
  if( BootLEDTime >= 500 )
      {
      if( BootLEDTime >= 1000 )
          {
          BootLEDTime = 0 ;
          Boot_LED(0) ;
          }
      else
          if( BootLEDTime == 500 )
              Boot_LED(1) ;
      }
  
  return ;
  */
}

//------------------------------------------------------------------------------
#define _FullFunction_                        1
//#define _DisablePowerOnRunLoader_             1
void main(void) 
{
  //#ifdef  _FullFunction_
  #ifndef _DisablePowerOnRunLoader_
  void (*JumpFunc)(void) ;
  unsigned long *Addr ;
  #endif
  //#endif
  //
  Boot_NVIC_SetVectorTable(NVIC_VectTab_FLASH,0) ;
  /* Configure one bit for preemption priority */
  Boot_NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  //
  Boot_HardwareInitial() ;
  //
#ifdef  _FullFunction_
  if( LoaderIntoMode != _SOFTWARE_ERR_ &&  LoaderIntoMode != _LOADER_ERR_ && LoaderIntoMode != _NO_USER_API_ )
      {
      LoaderIntoMode = 0 ;
      //----------------------------------------------------------------------------
      // Check LCB Program Exist

      if( Boot_CheckUserProgram() == _UserAPIOk_ )
          {
          Boot_DisableIRQn(1) ;
          Boot_CallApplication(APP_START_ADDRESS) ;
          }
      }
  //----------------------------------------------------------------------------
  #ifndef _DisablePowerOnRunLoader_
  //----------------------------------------------------------------------------
  // Check Loader Exist
  if( LoaderIntoMode != _LOADER_ERR_ )
      {
      if( Boot_CheckLoaderProgram() == _LoaderAPIOk_ )
          {
          LoaderIntoMode = _NO_USER_API_ ;
          Addr =(unsigned long *)(0x08000000 + _LInfo_LoaderStartAddress_) ;
          JumpFunc = (void(*)(void))(*Addr) ;
          JumpFunc() ;
          }
      }
  #endif
  //----------------------------------------------------------------------------
#endif
  Boot_main() ;
}








#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


