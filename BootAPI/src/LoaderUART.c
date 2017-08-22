/**
  ******************************************************************************
  * @file    LoaderUART.c
  * @author  JHT Team
  * @version V1.0.0
  * @date    03/12/2010
  * @brief   
  ******************************************************************************
*/ 

//------------------------------------------------------------------------------
#include  "LoaderUART.h"
#include  "stm32f10x.h"
#include  "LoaderUARTCommand.h"
#include  "Loader.h"


//------------------------------------------------------------------------------
// External 
extern __root const unsigned char Loader_Manufacture[40] ;
extern __root const unsigned char Loader_McuType[20];
extern __root const unsigned char Loader_ModuleName[20] ;
extern __root const unsigned char Loader_ModuleNo[20] ;
extern __root const unsigned char Loader_Product[20];
extern __root const unsigned char Loader_Version[20] ;
extern __root const unsigned char Loader_UnlockPassword[20] ;
extern __root const unsigned char User_CheckCode[16] ;
extern __root const unsigned char User_ProgramStatus ;
//------------------------------------------------------------------------------
extern void Loader_EnableRx(unsigned char Mode) ;
/* Private typedef -----------------------------------------------------------*/


typedef union {
  struct{
    unsigned char Ready:1 ;
    unsigned char Rx:1 ;
    unsigned char RxStart:1 ;
    unsigned char RxSFC:1 ;
    unsigned char RxOK:1 ;
    unsigned char Tx:1 ;
    unsigned char TxStart:1 ;
    unsigned char TxOK:1 ;
  } bit ;
  unsigned char All ;
} COMM_STATUS ;


struct FlashAddrByte
{
	unsigned long		Byte1:8 ;
	unsigned long 	Byte2:8 ;
	unsigned long		Byte3:8 ;
	unsigned long		Byte4:8 ;
} ;


typedef union
{
	struct FlashAddrByte Bit8 ;
	unsigned long		All ;
} Bit32 ;

/* Private define ------------------------------------------------------------*/
#define _OK_                    0xAA
#define _ERROR_                 0xFF

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
__no_init unsigned short COMM_TxRxDataIndex ;
__no_init unsigned char COMM_TxRxBuffer[_MaxBufferLength_] ;
__no_init volatile COMM_STATUS CommStatus ;
__no_init unsigned char	FlashUnLock ;	
volatile __no_init unsigned short DelayReturn ;
__no_init unsigned long FlashAdderssCheck ; // 20130524

/* Private functions ---------------------------------------------------------*/
unsigned char LoaderUART_ConvertSpecialData( unsigned char *pData, unsigned char InData ) ;

unsigned char LoaderUART_CheckRXD(void) ;
void LoaderUART_Decoder(void) ;
void LoaderUART_ReturnProductID(void) ;
void LoaderUART_ReturnMcuType(void) ;
void LoaderUART_ReturnMcuID(void) ;
void LoaderUART_RerurnUpdateMode(void) ;
void LoaderUART_ReturnFlashUnlock(void) ;
void LoaderUART_ReturnEraseStatus(void) ;
unsigned char LoaderUART_BlockCheck(unsigned long Addr,unsigned long Length) ;
void LoaderUART_ReturnCommandError(void) ;
void LoaderUART_ReturnWriteStatus(void) ;
void LoaderUART_ReturnReadStatus(void) ;
void LoaderUART_ReturnWriteCheckCode(void) ;
void LoaderUART_ReturnReadProgramState(void) ;	
unsigned char LoaderUART_CRC8( unsigned char *, unsigned long ) ;
void LoaderUART_UartWrite(unsigned char Mode,unsigned char Data) ;
void LoaderUART_ReturnLoaderVersion(void) ;
void LoaderUART_DealyTime(void) ;

/* Functions Source ----------------------------------------------------------*/
//#define   _DebugFlashWriteCommand     1
#ifdef  _DebugFlashWriteCommand
#define   _MaxDebugBuffer             255

__no_init unsigned short TestWrCount ;
__no_init unsigned long AddrTemp[_MaxDebugBuffer] ;
__no_init unsigned short CheckCT ;
#endif
//------------------------------------------------------------------------------
void LoaderUART_Initial(void)
{
  unsigned long integerdivider = 0 ;
  unsigned long fractionaldivider = 0 ;
  unsigned long tmpreg = 0 ;
  
  // Parameter Initial 
  CommStatus.All = 0 ;
  FlashUnLock = 0 ;
  COMM_TxRxDataIndex = 0 ;
  for( tmpreg=0 ; tmpreg < _MaxBufferLength_ ; tmpreg++)
      COMM_TxRxBuffer[tmpreg] = 0 ;  
  
  // 20130524
  #ifdef  _DebugFlashWriteCommand
  TestWrCount = 0 ;
  for(CheckCT=0;CheckCT<_MaxDebugBuffer;CheckCT++)
      AddrTemp[CheckCT]= 0 ;
  CheckCT = 0 ;
  #endif
  FlashAdderssCheck = 0 ; // 20130524
  //----------------------------------------------------------------------------
  // Work Length = 8 ;
  // Stop bits = 1 ;
  // No parity  
/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USART2->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= 0xCFFF;
  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (unsigned long)USART_StopBits_1 ;
  
  /* Write to USART CR2 */
  USART2->CR2 = (unsigned int)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USART2->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= 0xE9F3;
  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (unsigned long)USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx | USART_Mode_Tx ;
  /* Write to USART CR1 */
  USART2->CR1 = (unsigned int)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/  
  tmpreg = USART2->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= 0xFCFF;
  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_HardwareFlowControl_None ;
  /* Write to USART CR3 */
  USART2->CR3 = (unsigned int)tmpreg;

  /*---------------------------- USART BRR Configuration -----------------------*/  
  // Set BaudRate 115200
  // Baud = Fclk / ( 16 x BRR )
  // BRR = Fclk / ( 16 x Buad ) 
  //
  // Integer part computing in case Oversampling mode is 16 Samples 
  integerdivider = ((25 * 24000000 ) / (4 * 115200)) ;    
  tmpreg = (integerdivider / 100) << 4;
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  // Implement the fractional part in the register 
  tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((unsigned char)0x0F);
  //Write to USART BRR 
  USART2->BRR = (uint16_t)tmpreg;
  // 
  //USART_IT_TC,USART_IT_RXNE,Enable
  //
  USART2->CR1 |= 0x2060;  
  /* Compute the Corresponding IRQ Priority --------------------------------*/    
  NVIC->IP[USART2_IRQn] = 0;
  // Enable NVIC (53)
  /* Enable the Selected IRQ Channels --------------------------------------*/
  NVIC->ISER[USART2_IRQn >> 0x05] = (unsigned long)0x01 << (USART2_IRQn & (unsigned char)0x1F);
  //
  CommStatus.bit.Rx = 1 ;
  return ; 
}


//##############################################################################
// Function : 
// Input : none
// Return : none
//##############################################################################
void LoaderUART_TxRx_Information(void)
{
  unsigned char TempData ;

  // 1 1 1 1 1 1
  // 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
  //             C L T T R I O N F P 
  //             T B X C X D R E E E
  //             S D E   N L E
  //                     E E
  if( (USART2->SR & 0x00000040 ) != 0 ) 
      {
      // Clear the UART5 transmit interrupt
      USART2->SR &= 0xFFFFFFBF ;
      //
      CommStatus.bit.Tx = 0 ;
      /*
      if( CommStatus.bit.Tx == 1 )
          {
          // Transmit Data 
          USART2->DR = (COMM_TxRxBuffer[COMM_TxRxDataIndex] & (uint16_t)0x01FF);

          if( COMM_TxRxBuffer[COMM_TxRxDataIndex] == LCB_END )
              {
              CommStatus.bit.Tx = 0 ;
              CommStatus.bit.Rx = 1 ;
              CommStatus.bit.RxOK = 0  ;
              }
          else
              {
              COMM_TxRxDataIndex += 1 ;
              if( COMM_TxRxDataIndex >= _MaxBufferLength_ )
                  COMM_TxRxDataIndex = 0 ;      
              }
          //

          }
      */
      }
  //
  // 1 1 1 1 1 1
  // 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
  //             C L T T R I O N F P 
  //             T B X C X D R E E E
  //             S D E   N L E
  //                     E E  
  if( (USART2->SR & 0x00000020 ) != 0 ) 
      {
      TempData = (uint16_t)(USART2->DR & (uint16_t)0x01FF);
      // Clear the USART5 Receive interrupt 
      USART2->SR &= 0xFFFFFFDF ;
      if( CommStatus.bit.Rx == 1 )
          {
          switch(TempData)
              {
              case  LCB_START :
                              COMM_TxRxDataIndex = 0 ;
                              CommStatus.bit.RxStart = 1 ;
                              break ;
              case  LCB_SCF   :
                              CommStatus.bit.RxSFC = 1 ;
                              break ;
              case  LCB_END   :
                              if( CommStatus.bit.RxStart == 1 )
                                  {
                                  CommStatus.bit.RxOK = 1 ;
                                  CommStatus.bit.RxStart = 0 ;
                                  CommStatus.bit.Rx = 0 ;
                                  }                             
              default         :
                              if( CommStatus.bit.RxSFC == 1 )
                                  {
                                  if( TempData <= 3 )
                                      COMM_TxRxBuffer[COMM_TxRxDataIndex] = TempData + LCB_SCF ;
                                  else
                                      COMM_TxRxBuffer[COMM_TxRxDataIndex] = TempData ;
                                  }
                              else
                                  {
                                  COMM_TxRxBuffer[COMM_TxRxDataIndex] = TempData ;
                                  }
                              COMM_TxRxDataIndex += 1 ;
                              CommStatus.bit.RxSFC = 0 ;
                              if( COMM_TxRxDataIndex >= _MaxBufferLength_ )
                                  COMM_TxRxDataIndex = 0 ;   
                              break;
              }
          }
      //
      }
  //
  return ;
}


//##############################################################################
// Function : Convert 0xD1,0xD2,0xD0,D3 data to 0xD0+0x00,0xD0+0x01,0xD0+0x02
// Input : 
//        pData: data save memory address
//        InData: Convert Data
// Return : data count
//##############################################################################
unsigned char LoaderUART_ConvertSpecialData( unsigned char *pData, unsigned char InData )
{
  unsigned char Result ;
  Result = 1 ;
  if( InData == LCB_START || InData == LCB_END || InData == LCB_SCF )
      { 
      *pData = LCB_SCF ;
      *(pData+1) = InData & 0x0f ;
      Result = 2 ;
      }
  else
      {
      *pData = InData ;
      }
  return Result ;
}

//#############################################################################
// Check Communication Receiver data
//#############################################################################
void LoaderUART_UartWrite(unsigned char Mode,unsigned char Data)
{
  if( Mode == 0 )
      {
      CommStatus.bit.Tx = 1 ;
      USART2->DR = Data ;
      while(CommStatus.bit.Tx == 1) ;
      }
  else
      {
      if( Data == LCB_START || Data == LCB_END || Data == LCB_SCF || Data == LCB_REV )
          { 
          CommStatus.bit.Tx = 1 ;
          USART2->DR = LCB_SCF ;
          while(CommStatus.bit.Tx == 1) ;
          CommStatus.bit.Tx = 1 ;
          USART2->DR = Data & 0x0f ;
          while(CommStatus.bit.Tx == 1) ;          
          }
      else
          {
          CommStatus.bit.Tx = 1 ;
          USART2->DR = Data ;
          while(CommStatus.bit.Tx == 1) ;
          }
      }
  //
  return ;
}


//#############################################################################
// Check Communication Receiver data
//#############################################################################
unsigned char LoaderUART_CheckRXD()
{
	unsigned char  Status ;
	//
	switch( COMM_TxRxBuffer[0] )
			{
			case	_EraseFlash_				:		
			case	_WriteFlash_				:	
			case	_ReadFlash_					:	
			case	_ReadProgramState_	:					
			case	_FlashUnlock_				:	
			case	_WriteCheckCode_		:		
			case	_ReadProduceID_			:		
			case	_ReadMcuType_				:		
			case	_ReadMcuID_					:		
			case	_ReadUpdateMode_		:	
			case	_IsRequestUpdate_		:
      case  _ReadLoaderVersion_ :
																Status = _OK_ ;
																break ;
			default										:
																Status = _ERROR_ ;
																break ;			
			}
	//	
	return Status ;
}


//##############################################################################
void LoaderUART_RxProcess(void)
{
  unsigned long DelaytimeCT ;
  if( CommStatus.bit.RxOK == 1 )
      {
      if( LoaderUART_CheckRXD() == _OK_ )
          {
          CommStatus.bit.Rx = 0 ;
          /*
          DelayReturn = 500 ;
          while( DelayReturn > 0 ) ;
          */
          for( DelaytimeCT = 0 ; DelaytimeCT < 1000000 ; DelaytimeCT++) ;
          Loader_EnableRx(1) ;
          switch( COMM_TxRxBuffer[0] )
              {
              case	_EraseFlash_				:		
                                        LoaderUART_ReturnEraseStatus() ;
                                        break ;
              case	_WriteFlash_				:	
                                        if( FlashUnLock == 1 )
                                            LoaderUART_ReturnWriteStatus() ;
                                        break ;
              case	_ReadFlash_					:	
                                        if( FlashUnLock == 1 )
                                            LoaderUART_ReturnReadStatus() ;
                                        break ;
              case	_ReadProgramState_	:			
                                        if( FlashUnLock == 1 )
                                            LoaderUART_ReturnReadProgramState() ;		
                                        break ;
              case	_FlashUnlock_				:	
                                        LoaderUART_ReturnFlashUnlock() ;
                                        break ;
              case	_WriteCheckCode_		:	
                                        if( FlashUnLock == 1 )
                                            LoaderUART_ReturnWriteCheckCode() ;	
                                        break ;
              case	_ReadProduceID_			:	
                                        LoaderUART_ReturnProductID() ;	
                                        break ;
              case	_ReadMcuType_				:		
                                        LoaderUART_ReturnMcuType() ;
                                        break ;
              case	_ReadMcuID_					:		
                                        LoaderUART_ReturnMcuID() ;
                                        break ;
              case	_ReadUpdateMode_		:	
                                        LoaderUART_RerurnUpdateMode() ;
                                        break ;
              case  _ReadLoaderVersion_ :
                                        LoaderUART_ReturnLoaderVersion() ;
                                        break ;
              default										:
                                        break ;			
              }
          }
      else
          LoaderUART_ReturnCommandError() ;
      //
      Loader_EnableRx(0) ;
      //
      COMM_TxRxBuffer[0] = 0  ;
      CommStatus.bit.RxOK = 0 ;
      CommStatus.bit.Rx = 1 ;
      //
      }
  return ;
}


//#############################################################################
// Return Product ID
//#############################################################################
void LoaderUART_ReturnProductID(void)
{
	unsigned char Index ;
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnProduceID_) ;
	for( Index = 0 ; Index < 20 ; Index++ )
			{
			if( Loader_Product[Index] != 0 )
					{
					
					LoaderUART_UartWrite(1,Loader_ModuleName[Index]) ;
					}
			else
					break ;		
			}
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}




//#############################################################################
// Return MCU Type
//#############################################################################
void LoaderUART_ReturnMcuType(void)
{
	unsigned char Index ;
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnMcuType_) ;
	for( Index = 0 ; Index < 20 ; Index++ )
			{
			if( Loader_McuType[Index] != 0 )
					{
					
					LoaderUART_UartWrite(1,Loader_McuType[Index]) ;
					}
			else
					break ;		
			}
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}



//#############################################################################
// Return Return MCU ID
//#############################################################################
void LoaderUART_ReturnMcuID(void)
{
	unsigned char Index ;
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnMcuID_) ;
	for( Index = 0 ; Index < 20 ; Index++ )
			{
			if( Loader_ModuleNo[Index] != 0 )
					{
					LoaderUART_UartWrite(1,Loader_ModuleNo[Index]) ;
					}
			else
					break ;		
			}
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}

//
void LoaderUART_ReturnLoaderVersion(void)
{
	unsigned char Index ;
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnMcuID_) ;
	for( Index = 0 ; Index < 20 ; Index++ )
			{
			if( Loader_Version[Index] != 0 )
					{
					LoaderUART_UartWrite(1,Loader_Version[Index]) ;
					}
			else
					break ;		
			}
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}

//#############################################################################
// Return Update Mode
//#############################################################################
void LoaderUART_RerurnUpdateMode(void)
{
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnUpdateMode_) ;
	LoaderUART_UartWrite(0,_UpdateMode_) ;
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}




//#############################################################################
// Return Flash Unlock
//#############################################################################
void LoaderUART_ReturnFlashUnlock(void)
{
	unsigned char Index ;
	unsigned char Status ;
	//
	Status = _LockMode_ ;
	//
	for( Index = 0 ; Index < 20 ; Index++ )
			{
			if( Loader_UnlockPassword[Index] != 0 )
					{
					if( Loader_UnlockPassword[Index] != COMM_TxRxBuffer[Index+1] )
							{
							break ;
							}
					}
			else
					{
					if( COMM_TxRxBuffer[Index+1] == LCB_END )
							{
							Status = _UnlockMode_ ;	
							FlashUnLock = 1 ;
              Loader_Unlock() ;        
							}
					else
							break ;								
					}					
			} 
	//
	// Return Status
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnFlashState_) ;
	LoaderUART_UartWrite(0,Status) ;
	LoaderUART_UartWrite(0,LCB_END) ;
	//
	return ;
}






//#############################################################################
// Return Earse Status
//#############################################################################
void LoaderUART_ReturnEraseStatus(void)
{
	unsigned char DataLength ;
	Bit32 StartAddr ;
	Bit32 Length ;
	unsigned char Status ;
  FLASH_Status FMStatus;
	
	Status = _EraseFail_ ;
	//
	if( FlashUnLock == 1 )
			{
      // 20130524
      #ifdef  _DebugFlashWriteCommand
      TestWrCount = 0 ;
      for(CheckCT=0;CheckCT<_MaxDebugBuffer;CheckCT++)
          AddrTemp[CheckCT]= 0 ;
      CheckCT = 0 ;
      #endif
      FlashAdderssCheck = 0 ; // 20130524
      //------------------------------------------------------------------------
			// Check Command Length
			for( DataLength = 0 ; DataLength < 10 ; DataLength++ )
					{
					if( COMM_TxRxBuffer[DataLength+1] == LCB_END )
							break ;
					}
			
			if( DataLength == 8 )
					{
					StartAddr.Bit8.Byte1 = COMM_TxRxBuffer[1] ;
					StartAddr.Bit8.Byte2 = COMM_TxRxBuffer[2] ;
					StartAddr.Bit8.Byte3 = COMM_TxRxBuffer[3] ;
					StartAddr.Bit8.Byte4 = COMM_TxRxBuffer[4] ;
			    //StartAddr.All += 0x08000000 ;
          
					Length.Bit8.Byte1 = COMM_TxRxBuffer[5] ;
					Length.Bit8.Byte2 = COMM_TxRxBuffer[6] ;
					Length.Bit8.Byte3 = COMM_TxRxBuffer[7] ;
					Length.Bit8.Byte4 = COMM_TxRxBuffer[8] ;
          //
          if( LoaderUART_BlockCheck(StartAddr.All,Length.All) == 0 ) // 20130524
              {
              //
              if( StartAddr.All >= LCB_INFO_Addr && StartAddr.All < (LCB_INFO_Addr+FLASH_PAGE_SIZE))
                  { 
                  FMStatus = Loader_ErasePage(StartAddr.All) ;
                  if( FMStatus == FLASH_COMPLETE )
                      Status = _EraseOK_   ;  
                  }
              else
                  {
                  if( StartAddr.All >= LCB_StartAddr && (StartAddr.All+Length.All) <= LCB_EndAddr )
                      {
                      FMStatus = Loader_EarseUserProgram(StartAddr.All,(StartAddr.All+Length.All)) ;
                      if( FMStatus == FLASH_COMPLETE )
                          Status = _EraseOK_   ;     
                      }
                  }
              }
          else
              Status = _EraseOK_  ; 
					}
			}
	//
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnEraseFlash_) ;
	LoaderUART_UartWrite(0,Status) ;
	LoaderUART_UartWrite(0,LCB_END) ;
	//
	return ;
}


unsigned char LoaderUART_BlockCheck(unsigned long Addr,unsigned long Length)
{
  unsigned char *ptr ;
  unsigned long Len ;
  
  ptr = (unsigned char*)Addr ;
  
  for( Len = 0 ; Len < Length ; Len++ )
      {
      if( *(ptr+Len) != 0xFF )
          return 0 ;
      }
  
  return 1 ;
}

//#############################################################################
// Return Command Error
//#############################################################################
void LoaderUART_ReturnCommandError(void)
{
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnInvalidCommand_) ;
	LoaderUART_UartWrite(0,LCB_END) ;
	return ;
}


//#############################################################################
// Return Write Status
//#############################################################################
void LoaderUART_ReturnWriteStatus(void)
{
  //
	Bit32 StartAddr ;
	Bit32 Length ;
	unsigned long Index ;
	unsigned char Crc8 ;
	FLASH_Status FMStatus;
  unsigned short *ptr ;
  unsigned char dT ;
	//
	StartAddr.Bit8.Byte1 = COMM_TxRxBuffer[1] ;
	StartAddr.Bit8.Byte2 = COMM_TxRxBuffer[2] ;
	StartAddr.Bit8.Byte3 = COMM_TxRxBuffer[3] ;
	StartAddr.Bit8.Byte4 = COMM_TxRxBuffer[4] ;
  //StartAddr.All += 0x08000000 ;
  // 20130524
  #ifdef  _DebugFlashWriteCommand
  AddrTemp[CheckCT]=StartAddr.All;
  CheckCT += 1 ;
  #endif
	//
	Length.All = COMM_TxRxDataIndex-6 ;
	//
  ptr = (unsigned short*)&COMM_TxRxBuffer[5] ;
  //
  if( FlashAdderssCheck != StartAddr.All ) // add by Kunlung 20130524
      {
      FlashAdderssCheck = StartAddr.All ;
      for( Index = 0 ; Index < (COMM_TxRxDataIndex-6) ; Index+=2 )
          {
          if( StartAddr.All >= LCB_StartAddr && (StartAddr.All+Length.All) <= LCB_EndAddr )
              {
              if( (StartAddr.All+Index) < 0x0801FFFF )
                  {
                  for( dT = 0 ; dT < 0xFF ; dT++ ) ;              
                  FMStatus = Loader_ProgramHalfWord((StartAddr.All+Index),*ptr ) ;              
                  if( FMStatus != FLASH_COMPLETE )
                      {
                      Length.All = Index ;
                      break ;
                      }
                  else
                      ptr++ ;
                  }
              else
                  {
                  Length.All = Index-2 ;
                  break ;
                  }   
              }
          else
              {
              Length.All = Index ;
              break ;
              }
          }
      // for test
      #ifdef  _DebugFlashWriteCommand
      if( Length.All == (COMM_TxRxDataIndex-6) )
          TestWrCount += 1 ;
      else
          TestWrCount = 0 ;
      #endif
      //
      }
	Crc8 = LoaderUART_CRC8( (unsigned char *)(StartAddr.All), Length.All ) ;
  //StartAddr.All -= 0x08000000 ;
	//
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnWriteFlash_) ;
	//
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte1) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte2) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte3) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte4) ;
	LoaderUART_UartWrite(1,Length.Bit8.Byte1) ;
	LoaderUART_UartWrite(1,Length.Bit8.Byte2) ;
	LoaderUART_UartWrite(1,Length.Bit8.Byte3) ;
	LoaderUART_UartWrite(1,Length.Bit8.Byte4) ;
	LoaderUART_UartWrite(1,Crc8) ;
	//
	LoaderUART_UartWrite(0,LCB_END) ;
  //
	return ;
}




unsigned char const LoaderUARTCrcTab[16]  =	{0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e};
//############################################################################
// Cyclic Redundant Check ( CRC-8 )
// ¦h¶µ¦¡(Polynomial) X8¡ÏX5¡ÏX4¡Ï1
//############################################################################
unsigned char LoaderUART_CRC8( unsigned char *ptr, unsigned long DATALENGTH )
{
  unsigned long crcheckcnt;
	unsigned char crchecktemp,crchalf,CHECKDATA;
	//
  CHECKDATA=0;
	//
  for(crcheckcnt =0; crcheckcnt < DATALENGTH; crcheckcnt++ )
    	{
      crchecktemp = *(ptr + crcheckcnt);
      crchalf =(CHECKDATA/16);
      CHECKDATA <<= 4;
			CHECKDATA &= 0x00FF ;
      CHECKDATA ^= LoaderUARTCrcTab[crchalf ^( crchecktemp / 16)];
  		CHECKDATA &= 0x00FF ;
      crchalf = (CHECKDATA/16);
      CHECKDATA <<= 4;
			CHECKDATA &= 0x00FF ;
      CHECKDATA ^= LoaderUARTCrcTab[crchalf ^( crchecktemp & 0x0f)];
      CHECKDATA &= 0x00FF ;
    	}
	//
  return CHECKDATA;
	//---------------------------------------------------------------------------
}



//#############################################################################
// Return Read Status
//#############################################################################
void LoaderUART_ReturnReadStatus(void)
{
	Bit32 StartAddr ;
	Bit32 Length ;
	unsigned char *ReturnPtr ;
	unsigned long Index ;
	
	StartAddr.Bit8.Byte1 = COMM_TxRxBuffer[1] ;
	StartAddr.Bit8.Byte2 = COMM_TxRxBuffer[2] ;
	StartAddr.Bit8.Byte3 = COMM_TxRxBuffer[3] ;
	StartAddr.Bit8.Byte4 = COMM_TxRxBuffer[4] ;
  
	Length.Bit8.Byte1 = COMM_TxRxBuffer[5] ;
	Length.Bit8.Byte2 = COMM_TxRxBuffer[6] ;
	Length.Bit8.Byte3 = COMM_TxRxBuffer[7] ;
	Length.Bit8.Byte4 = COMM_TxRxBuffer[8] ;
		
	//
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnReadFlash_) ;
	//
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte1) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte2) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte3) ;
	LoaderUART_UartWrite(1,StartAddr.Bit8.Byte4) ;
	//
  
	ReturnPtr = (unsigned char *)StartAddr.All;//+0x08000000 ;
	for( Index = 0 ; Index < Length.All ; Index++ )
			{
			LoaderUART_UartWrite(1,*ReturnPtr) ;
			ReturnPtr += 1 ;
			}
	
	//
	LoaderUART_UartWrite(0,LCB_END) ;
	//
	return ;
}




//#############################################################################
// Return Write Check Code
//#############################################################################
void LoaderUART_ReturnWriteCheckCode(void) 
{
	unsigned char *CheckPtr ;
  unsigned short *DataPtr ;
	unsigned char Index ;
  unsigned short ProgramStatus ;
  FLASH_Status FMStatus;
  // Add 20130524
  ProgramStatus = 0 ;
  for(Index = 0 ;Index < 16;Index++ )
      {
      if( User_CheckCode[Index] != 0xFF )
          {
          ProgramStatus = 1 ;
          break ;
          }      
      }
  //
  if( ProgramStatus == 0 ) // Add by KunLung 20130524
      {
      CheckPtr = (unsigned char *)&User_CheckCode[0] ;
      DataPtr = (unsigned short *)&COMM_TxRxBuffer[1] ;
      //
      for( Index = 0 ; Index < 16 ; Index+=2 )
          {
          FMStatus = Loader_ProgramHalfWord((unsigned long)(CheckPtr+Index),*DataPtr ) ; 
          if( FMStatus != FLASH_COMPLETE )
              {
              break ;
              }
          else
              DataPtr += 1 ;
          }
          
      if( Index >= 16 )
          {
          ProgramStatus = 0x01 ;
          CheckPtr = (unsigned char *)&User_ProgramStatus ;
          if( (*CheckPtr == 0xFF) && (*(CheckPtr+1) == 0xFF)  ) // 20130524
              Loader_ProgramHalfWord((unsigned long)CheckPtr,ProgramStatus ) ;
          }
      }
	//
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnCheckCode_) ;
	//
	for( Index = 0 ; Index < 16 ; Index++ )
			LoaderUART_UartWrite(1,User_CheckCode[Index]) ;
	//
	LoaderUART_UartWrite(0,LCB_END) ;
	//

	return ;
}






//#############################################################################
// Return Read Program State
//#############################################################################
void LoaderUART_ReturnReadProgramState(void)
{
	//
	LoaderUART_UartWrite(0,LCB_START) ;
	LoaderUART_UartWrite(0,_ReturnProgramState_) ;
	LoaderUART_UartWrite(1,(unsigned char)User_ProgramStatus) ;
	LoaderUART_UartWrite(0,LCB_END) ;
	//
	return ;
}


void LoaderUART_DealyTime(void)
{
  if( DelayReturn != 0 )
      DelayReturn -= 1 ;
  return ;
}






