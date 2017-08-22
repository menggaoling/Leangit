/**
  ******************************************************************************
  * @file    Loader.c
  * @author  JHT Team
  * @version V1.0.0
  * @date    03/12/2010
  * @brief   
  ******************************************************************************
*/ 

//------------------------------------------------------------------------------
#include  "stm32f10x.h"
#include  "Loader.h"
#include  "LoaderUart.h"


//------------------------------------------------------------------------------
//#define   _NO_USER_API_               0x5A
//extern __no_init unsigned char LoaderIntoMode ;


//------------------



//------------------------------------------------------------------------------

__root void Loader_Start(void) ;
//__root void Loader_main(void) @ ".loaderstart" ;

// array size ¥²¶·¬O¥|ªº­¿¼Æ
__root void * const LoaderStartAddress                  @ ".loaderinfo" = (void *)Loader_Start ;
__root const unsigned char Loader_Manufacture[40]       @ ".loaderinfo" = "Johnson Health Tech. Co., Ltd." ;
__root const unsigned char Loader_McuType[20]           @ ".loaderinfo" = "STM32F102RB" ;
__root const unsigned char Loader_ModuleName[20]        @ ".loaderinfo" = "1xBikeLCB";//"LCB-A" ;
__root const unsigned char Loader_ModuleNo[20]          @ ".loaderinfo" = "L13CA1";//"L118A1_MCU-B" ;
__root const unsigned char Loader_Product[20]           @ ".loaderinfo" = "Matrix" ;
__root const unsigned char Loader_Version[20]           @ ".loaderinfo" = "V001-20140425";//"S001-20130415" ;
__root const unsigned char Loader_UnlockPassword[20]    @ ".loaderinfo" = "Johnson" ;
__root const unsigned short LCB_LoaderVersion           @ ".loaderinfo" = 0x1365 ;
__root const unsigned short Loader_ProgramStatus        @ ".loaderinfo" = 0xB65A ;
__root const unsigned long  Loader_MCUFlashOffset       @ ".loaderinfo" = 0x08000000 ; 
__root const unsigned long  Loader_UserAPIStartAddr     @ ".loaderinfo" = 0x08006000 ; 
__root const unsigned long  Loader_UserAPIEndAddr       @ ".loaderinfo" = 0x0801FBFF ; 
__root const unsigned long  Loader_UserInfoStartAddr    @ ".loaderinfo" = 0x0801FC00 ; 
__root const unsigned long  Loader_UserInfoEndAddr      @ ".loaderinfo" = 0x0801FFFF ; 
__root const unsigned long  Loader_LAPIStartAddr        @ ".loaderinfo" = 0x08003C00 ; 
__root const unsigned long  Loader_LAPIEndAddr          @ ".loaderinfo" = 0x08005BFF ; 
__root const unsigned long  Loader_LInfoStartAddr       @ ".loaderinfo" = 0x08005C00 ; 
__root const unsigned long  Loader_LInfoEndAddr         @ ".loaderinfo" = 0x08005FFF ;

//------------------------------------------------------------------------------
__no_init unsigned long LEDTime ;




void Loader_LED(unsigned char Data) ;
void Loader_SysTick(void) ;
void Loader_EnableRx(unsigned char Mode) ;

//##############################################################################
void Loader_Start(void)
{
  // Disable All 
  SCB->SHCSR &= ~SCB_SHCSR_SYSTICKACT ;
  SysTick->VAL   = 0 ;                      /* Load the SysTick Counter Value */
  SysTick->CTRL  = 0 ;
  NVIC->ICER[0] = 0xFFFFFFFF ;
  NVIC->ICER[1] = 0xFFFFFFFF ;
  NVIC->ICER[2] = 0xFFFFFFFF ;
  //
  // Set the vector table to the beginning of the app in flash.
  //
  SCB->VTOR = NVIC_VectTab_LOADER ;
  //
  // Load the stack pointer from the application's vector table.
  //
  __asm("    ldr     r0, [r1]\n"
        "    mov     sp, r0");

  //
  // Load the initial PC from the application's vector table and branch to
  // the application's entry point.
  //
  __asm("    ldr     r1, [r1, #4]\n"
        "    bx      r1\n");
  
  return ;
}


//##############################################################################
void Loader_LED(unsigned char Data)
{
  if( Data == 0 )
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0011 1100 0000
      // 0000 0000 0000 0000 1000 0011 1000 0000
      //GPIOB->BRR =  0x00008380 ; 
      GPIOA->BRR = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//|GPIO_Pin_12|GPIO_Pin_11;  
      GPIOC->BRR =  GPIO_Pin_9 ; // 1x LCB
      }
  else
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0011 1100 0000
      // 0000 0000 0000 0000 1000 0011 1000 0000
      //GPIOB->BSRR =  0x00008380 ;
      GPIOA->BSRR = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;//|GPIO_Pin_12|GPIO_Pin_11;  
      GPIOC->BSRR =  GPIO_Pin_9 ; // 1x LCB  
      }
  return ;
}


//##############################################################################
void Loader_SysTick(void)
{
  LEDTime += 1 ;
  if( LEDTime >= 500 )
      {
      if( LEDTime >= 1000 )
          {
          LEDTime = 0 ;
          Loader_LED(0) ;
          }
      else
          if( LEDTime == 500 )
              Loader_LED(1) ;
      }
  return ;
}

//##############################################################################
// PA1 : Output, RS485 DE/RE
void Loader_EnableRx(unsigned char Mode)
{
  if( Mode == 0 )
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0000 0000 0010
      GPIOA->BRR =  0x00000002 ; 
      }
  else
      {
      // FEDC BA98 7654 3210 FEDC BA98 7654 3210
      // 0000 0000 0000 0000 0000 0000 0000 0010
      GPIOA->BSRR =  0x00000002 ;
      }
  return ;
}



//##############################################################################
//
//##############################################################################
void Loader_main(void)
{
  // Disable All 
  SCB->SHCSR &= ~SCB_SHCSR_SYSTICKACT ;
  SysTick->VAL   = 0 ;                      /* Load the SysTick Counter Value */
  SysTick->CTRL  = 0 ;
  NVIC->ICER[0] = 0xFFFFFFFF ;
  NVIC->ICER[1] = 0xFFFFFFFF ;
  NVIC->ICER[2] = 0xFFFFFFFF ;
  // Initial Memory
  LEDTime = 0 ;
  // Set the vector table to the beginning of the app in flash.
  SCB->VTOR = NVIC_VectTab_LOADER ;
  SCB->AIRCR = 0x05FA0000 | NVIC_PriorityGroup_4;
  
  /* for Testing Mask by Kunlung 20130523
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
  // Initial Uart
  LoaderUART_Initial() ;
  //
  GPIOA->BRR = GPIO_Pin_12|GPIO_Pin_11 ; // VCC3.3
  GPIOC->BRR = GPIO_Pin_10 ; // Power ON
  //
  for(;;)
      {
      //------------------------------------------------------------------------
      LoaderUART_RxProcess() ;  
      //------------------------------------------------------------------------
      // Add by Kunlung 20130523
      LEDTime += 1 ;
      if( LEDTime >= 100000 )
          {
          if( LEDTime >= 200000 )
              {
              LEDTime = 0 ;
              Loader_LED(0) ;
              }
          else
              if( LEDTime == 100000 )
                  Loader_LED(1) ;
          }
      //
      }
  //
}



/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void Loader_Unlock(void)
{
  /* Authorize the FPEC Access */
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void Loader_Lock(void)
{
  /* Set the Lock Bit to lock the FPEC and the FCR */
  FLASH->CR |= CR_LOCK_Set;
}

/**
  * @brief  Erases a specified FLASH page.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status Loader_ErasePage(uint32_t Page_Address)
{
  unsigned long *ptr ;
  unsigned short count ;
  
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Page_Address));
  /* Wait for last operation to be completed */
  status = Loader_WaitForLastOperation(EraseTimeout);
  
  if(status == FLASH_COMPLETE)
  { 
    /* if the previous operation is completed, proceed to erase the page */
    FLASH->CR|= CR_PER_Set;
    FLASH->AR = Page_Address; 
    FLASH->CR|= CR_STRT_Set;
    
    /* Wait for last operation to be completed */
    status = Loader_WaitForLastOperation(EraseTimeout);
    //if(status != FLASH_TIMEOUT)
    //{
    /* if the erase operation is completed, disable the PER Bit */
    FLASH->CR &= CR_PER_Reset;
    // Check Block
    ptr = (unsigned long *)Page_Address ;
    for( count = 0 ; count < (FLASH_PAGE_SIZE/4) ; count++ )
        {
        if( *(ptr+count) != 0xFFFFFFFF )
            {
            status = FLASH_ERROR_PG ;
            break ;
            }
        }
    //
    //}
  }
  /* Return the Erase Status */
  return status;
}



/**
  * @brief  Programs a half word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status Loader_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;
  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));
  /* Wait for last operation to be completed */
  status = Loader_WaitForLastOperation(ProgramTimeout);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR |= CR_PG_Set;
  
    *(__IO uint16_t*)Address = Data;
    /* Wait for last operation to be completed */
    status = Loader_WaitForLastOperation(ProgramTimeout);
    //if(status != FLASH_TIMEOUT)
    //{
      /* if the program operation is completed, disable the PG Bit */
      FLASH->CR &= CR_PG_Reset;
    //}
  } 
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Clears the FLASH’s pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:         
  *     @arg FLASH_FLAG_PGERR: FLASH Program error flag       
  *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag      
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag           
  * @retval None
  */
void Loader_ClearFlag(uint16_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG)) ;
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status Loader_GetStatus(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & FLASH_FLAG_PGERR) != 0)
    { 
      flashstatus = FLASH_ERROR_PG;
    }
    else 
    {
      if((FLASH->SR & FLASH_FLAG_WRPRTERR) != 0 )
      {
        flashstatus = FLASH_ERROR_WRP;
      }
      else
      {
        flashstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the Flash Status */
  return flashstatus;
}


/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH progamming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status Loader_WaitForLastOperation(uint32_t Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the Flash Status */
  status = Loader_GetStatus();
  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
  {
    status = Loader_GetStatus();
    Timeout--;
  }
  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

/**
  * @}
  */
FLASH_Status Loader_EarseUserProgram(uint32_t SAddr,uint32_t EAddr)
{ 
  FLASH_Status status = FLASH_COMPLETE;
  uint32_t DataCounter = 0x00 ;
  __IO uint32_t NbrOfPage = 0x00;
  //
 
  /* Define the number of page to be erased */
  NbrOfPage = (EAddr - SAddr + 1) / FLASH_PAGE_SIZE;
  
  /* Clear All pending flags */
  Loader_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
  status = FLASH_COMPLETE ;
  /* Erase the FLASH pages */
  for(DataCounter = 0; (DataCounter < NbrOfPage) && (status == FLASH_COMPLETE); DataCounter++)
      {
      if( status == FLASH_COMPLETE )
          status = Loader_ErasePage(SAddr + (FLASH_PAGE_SIZE * DataCounter));
      else
          break ;      
      }
  
  /* Return the operation status */
  return status;
}



//##############################################################################
FLASH_Status Loader_EarseLCBProgram()
{
  FLASH_Status status;
  //----------------------------------------------------------------
  /* Unlock the Flash Program Erase controller */
  Loader_Unlock();
  // Earse Flash
  status = Loader_EarseUserProgram(LCB_StartAddr,LCB_EndAddr) ;
  // 
  return status ;
}






