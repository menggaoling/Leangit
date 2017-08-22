/**
  ******************************************************************************
  * @file    Bootuart.c
  * @author  JHT Team
  * @version V1.0.0
  * @date    03/12/2010
  * @brief   
  ******************************************************************************
*/ 

//------------------------------------------------------------------------------
#include  "Bootuart.h"
#include  "stm32f10x.h"
//#include  "BootuartCommand.h"
#include  "JHTCOMMAND.H"
#include  "Boot.h"
#include  "Bootflash.h"
#include  "Loader.h"


#define   LCB_LoaderStartAddr               ((uint32_t)0x08003C00)
#define   LCB_LoaderEndAddr                 ((uint32_t)0x08005FFF)
#define   LCB_LoaderMarkStartAddr           (LCB_LoaderEndAddr-FLASH_PAGE_SIZE+1)  

typedef union {
  struct {
    unsigned char Start ;
    unsigned char Status ;
    unsigned char Command ;
    unsigned char Length ;
    unsigned char Data[30] ;
  } member ;
  unsigned char Buffer[34] ;
} BootuartTxDataStruct ;

typedef union {
  struct {
    unsigned char Start ;
    unsigned char Address ;
    unsigned char Command ;
    unsigned char Length ;
    unsigned char Data[50] ;
  } member ;
  unsigned char Buffer[54] ;
} BootuartRxDataStruct ;


typedef union {
  struct{
    unsigned short Ready:1 ;
    unsigned short Rx:1 ;
    unsigned short RxStart:1 ;
    unsigned short RxEndWait5msReturn:1 ;
    unsigned short RxOK:1 ;
    unsigned short Tx:1 ;
    unsigned short TxStart:1 ;
    unsigned short TxOK:1 ;
  } bit ;
  unsigned short All ;
} Bootuart_STATUS ;




Bootuart_STATUS BootuartStatus ;

// 20130411
unsigned char JustIntoUpdateAPI = 0 ;
#define   _WaitIntoUpdate             0x2A
#define   _JustIntoUpdate             0x3A
//------------------------------------------------------------------------------


#define BootuartRxSize        200 
unsigned char BootuartRxBuffer[BootuartRxSize] ;
unsigned char *BootuartRxStartPoint; 
unsigned char *BootuartRxEndPoint;
// 20120204 Add Tx FIFO Control
#define BootuartTxSize        200 
unsigned char BootuartTxBuffer[BootuartTxSize] ;
unsigned char *BootuartTxStartPoint; 
unsigned char *BootuartTxEndPoint;
unsigned short BootuartTxDataPoint ;
unsigned short BootuartTxDataLength ;
BootuartRxDataStruct BootuartRxAnalsys ;
BootuartTxDataStruct BootuartTxData ;


volatile unsigned short BootuartTxDelayTimeCounter ;



void Bootuart_RxFunc_Init(void) ;
unsigned char* Bootuart_RxFunc_NextPt(unsigned char *pt) ;
unsigned char Bootuart_RxFunc_IsEmpty(void) ;
unsigned char Bootuart_RxFunc_IsFull(void) ;
unsigned short Bootuart_RxFunc_Length(void) ;
void Bootuart_RxFunc_Putc(unsigned char c) ;
unsigned char Bootuart_RxFunc_Getc(void) ;
unsigned char Bootuart_CRC8( unsigned char *ptr, unsigned short DATALENGTH ) ;
void Bootuart_CommandDecoder(unsigned char ECmd ) ;
unsigned char Bootuart_CheckCommandAndLength(unsigned char CheckCmd,unsigned char SubCmd ,unsigned char CheckLen ) ;
void Bootuart_ReturnData( unsigned char TransCmd, unsigned char *DataPtr, unsigned char DataLen ) ;

//------------------------------------------------------------------------------
void  Bootuart_Initial(void)
{
  unsigned long integerdivider = 0 ;
  unsigned long fractionaldivider = 0 ;
  unsigned long tmpreg = 0 ;
  
  Bootuart_RxFunc_Init() ; 
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
  // Set BaudRate 9600
  // Baud = Fclk / ( 16 x BRR )
  // BRR = Fclk / ( 16 x Buad ) 
  //
  // Integer part computing in case Oversampling mode is 16 Samples 
  integerdivider = ((25 * 24000000 ) / (4 * 9600)) ;    
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
  // Parameter Initial 
  BootuartStatus.All = 0 ;
  BootuartTxDelayTimeCounter = 0 ;
  Bootuart_EnableRx(_RXD) ;
  // 20130411
  JustIntoUpdateAPI = 0 ;
  return ; 
}


//##############################################################################
// Function : 
// Input : none
// Return : none
//##############################################################################
void Bootuart_TxRx_Information(void)
{
  unsigned char TempData ;
  // TXD
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
      if( BootuartStatus.bit.Tx == 1 )
          {
          // Transmit Data 
          if( BootuartTxDataPoint < BootuartTxDataLength )
              {           
              USART2->DR = (BootuartTxData.Buffer[BootuartTxDataPoint] & (uint16_t)0x01FF);
              BootuartTxDataPoint += 1 ;
              }
          else
              {
              Bootuart_EnableRx(_RXD) ;
              BootuartTxDataPoint = 0 ;
              BootuartStatus.bit.Tx = 0 ;
              BootuartStatus.bit.Rx = 1 ;
              if( JustIntoUpdateAPI == _WaitIntoUpdate )
                  JustIntoUpdateAPI = _JustIntoUpdate ;
              }
          //
          }
      }
  // RXD
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
      Bootuart_RxFunc_Putc(TempData) ;
      //
      }
  //
  return ;
}




/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void Bootuart_RxFunc_Init(void)
{
  BootuartRxStartPoint = BootuartRxEndPoint = &BootuartRxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char* Bootuart_RxFunc_NextPt(unsigned char *pt)
{
  return ((pt - &BootuartRxBuffer[0]) < (BootuartRxSize-1))?(pt+1):&BootuartRxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char Bootuart_RxFunc_IsEmpty(void)
{
  return (BootuartRxStartPoint == BootuartRxEndPoint)?1:0;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char Bootuart_RxFunc_IsFull(void)
{
  return (BootuartRxStartPoint == Bootuart_RxFunc_NextPt(BootuartRxEndPoint))?1:0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned short Bootuart_RxFunc_Length(void)
{ 
	if(BootuartRxStartPoint <= BootuartRxEndPoint ) 
	    return BootuartRxEndPoint - BootuartRxStartPoint ; 
	else 
	    return BootuartRxSize - (BootuartRxStartPoint - BootuartRxEndPoint); 
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void Bootuart_RxFunc_Putc(unsigned char c)
{ 
	if(BootuartRxStartPoint == Bootuart_RxFunc_NextPt(BootuartRxEndPoint)) 
	    return ; 
	*BootuartRxEndPoint = c; 
	BootuartRxEndPoint = Bootuart_RxFunc_NextPt(BootuartRxEndPoint) ;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char Bootuart_RxFunc_Getc(void)
{ 
	unsigned char result=0; 
  
	if(BootuartRxEndPoint != BootuartRxStartPoint)
      { 
      result = *BootuartRxStartPoint; 
      BootuartRxStartPoint = Bootuart_RxFunc_NextPt(BootuartRxStartPoint) ;
	    } 
  
	return result;
} 



const unsigned char BootuartCrcTab[16]={0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e};
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char Bootuart_CRC8( unsigned char *ptr, unsigned short DATALENGTH )
{
  unsigned char crchecktemp,crchalf,CHECKDATA=0;
  unsigned short crcheckcnt = 0 ;
  //
  for(crcheckcnt =0; crcheckcnt < DATALENGTH; crcheckcnt ++)
      {
      crchecktemp = *(ptr+crcheckcnt);
      crchalf =(CHECKDATA/16);
      CHECKDATA<<=4;
      CHECKDATA^=BootuartCrcTab[crchalf ^( crchecktemp /16)];
      crchalf =(CHECKDATA/16);
      CHECKDATA<<=4;
      CHECKDATA^=BootuartCrcTab[crchalf ^( crchecktemp &0x0f)];
      }
  //
  return CHECKDATA;
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void Bootuart_RxProcess(void)
{
  //
  unsigned char *pt1;
	unsigned short i ;
  unsigned short Length ;
  unsigned char CheckSum ;
  //20130411
  void (*JumpFunc)(void) ;
  unsigned long *Addr ;
  //
  if( JustIntoUpdateAPI == _JustIntoUpdate )
      {
      Addr =(unsigned long *)(0x08000000 + _LInfo_LoaderStartAddress_) ;
      JumpFunc = (void(*)(void))(*Addr) ;
      JumpFunc() ;
      }
  //
  if( BootuartStatus.bit.RxEndWait5msReturn == 0 )
      {
      if( BootuartStatus.bit.Tx == 0 )
          {
          // Check Receive Buffer
          if(Bootuart_RxFunc_IsEmpty())
              {
              return ;
              }
            
          // Check Start Byte
          pt1 = BootuartRxStartPoint;
          if( *pt1 != 0x00 )    // UCB_START_BYTE
              {
              Bootuart_RxFunc_Getc();    // drop StartCode byte	
              return ;
              }
          //
          if( Bootuart_RxFunc_Length() > 3 )
              {
              //----------------------------------------------------------------
              BootuartRxAnalsys.member.Start = *pt1 ;
              pt1 = Bootuart_RxFunc_NextPt(pt1) ;
              //
              BootuartRxAnalsys.member.Address = *pt1 ;
              if( BootuartRxAnalsys.member.Address != 0xFF ) // UCB_ADDRESS_BYTE
                  {
                  Bootuart_RxFunc_Getc();    // drop StartCode byte	
                  return ;
                  }
              //
              pt1 = Bootuart_RxFunc_NextPt(pt1) ;
              BootuartRxAnalsys.member.Command = *pt1 ;
              pt1 = Bootuart_RxFunc_NextPt(pt1) ;
              BootuartRxAnalsys.member.Length = *pt1 ;
              pt1 = Bootuart_RxFunc_NextPt(pt1) ;
              Length = BootuartRxAnalsys.member.Length ;
              //----------------------------------------------------------------
              if( Bootuart_RxFunc_Length() >= (Length+5))
                  {
                  // Move Data to Buffer  
                  for( i = 0 ; i < Length ; i++ )
                      {
                      BootuartRxAnalsys.member.Data[i] = *pt1 ;
                      pt1 = Bootuart_RxFunc_NextPt(pt1) ;
                      }
                  //------------------------------------------------------------
                  Length += 4 ;
                  CheckSum = *pt1 ;
                  if( CheckSum == Bootuart_CRC8( &BootuartRxAnalsys.Buffer[0],Length ) )
                      {
                      //
                      //CommControlFlag.bit.RxTimeoutCheckStart = 0 ;
                      //RxTimeoutCounter = 0 ;
                      //RxDisconnectCounter = 0 ;
                      //
                      Bootuart_CommandDecoder( BootuartRxAnalsys.member.Command ) ;
                      // Clear buffer
                      for( i = 0 ; i < (Length+1) ; i++ )
                          {
                          Bootuart_RxFunc_Getc();    // drop StartCode byte
                          }
                      //
                      BootuartStatus.bit.RxEndWait5msReturn = 1 ;
                      BootuartStatus.bit.Tx = 1 ;
                      //--------------------------------------------------------
                      // Set Status LED Blink
                      //CommControlFlag.bit.StatusLED = ~CommControlFlag.bit.StatusLED ;
                      //--------------------------------------------------------
                      }
                  else
                      {
                      Bootuart_RxFunc_Getc();    // drop StartCode byte
                      BootuartStatus.bit.RxEndWait5msReturn = 0 ;
                      BootuartStatus.bit.Tx = 0 ;
                      }
                  }
              }
          /*
          #ifdef _EnableErrorCode_
          // Check Communication Disconnect
          if( RxDisconnectCounter > RxDisconnectTime )
              {
              RxDisconnectCounter = 0 ;
              ErrorCodeStatus.bit.EC04A0 = 1 ;
              }      
          #endif
          */
          //
          }
      }
  else
      {
      BootuartTxDelayTimeCounter += 1 ;
      if( BootuartTxDelayTimeCounter > 20000 )
          {
          BootuartStatus.bit.RxEndWait5msReturn = 0 ;
          Bootuart_EnableRx(_TXD) ;
          BootuartTxDelayTimeCounter = 0 ;
          BootuartTxDataPoint = 1 ;
          USART2->DR = (BootuartTxData.Buffer[0] & (uint16_t)0x01FF);
          }
      }
  return ;
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void Bootuart_CommandDecoder(unsigned char ECmd )
{
  unsigned char RetDataLength ;
  unsigned char RetData[255] ;
  unsigned char Command ;
  unsigned char SubCommand ;
  union {
    struct {
      unsigned short Low:8 ;
      unsigned short High:8 ;
    } By ;
    unsigned short All ;
  } TempData ;
  
  union {
    struct {
      unsigned long LL:8 ;
      unsigned long LH:8 ;
      unsigned long HL:8 ;
      unsigned long HH:8 ;
    } By ;
    unsigned long All ;
  } StartAddr,EndAddr ;
  
  union {
    struct {
      unsigned short MCU_A_Loader ;
      unsigned short MCU_A_API ;
      unsigned short MCU_B_Loader ;
      unsigned short MCU_B_API ;
    } Ver ;
    unsigned char VerData[8] ;
  } LCBVerInfo ;
  
  unsigned short *ptr ;
  unsigned short WLength ;
  unsigned char  *cptr ;
  unsigned char i ;
  // Default 0
  RetDataLength = 0 ;
  //
  Command = ECmd ;
  SubCommand = BootuartRxAnalsys.member.Data[0] ;
  if( Bootuart_CheckCommandAndLength(Command,SubCommand,BootuartRxAnalsys.member.Length) == 1 )
      {
      switch( Command )
          {
          case 	CmdInitial      										:	
          case 	CmdGetStatus    										:	
                                                    break ;
          case	CmdGetErrorCode 										:
                                                    TempData.All = 0 ;
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    break ;
          case	CmdSkipErrorCode 										:	
                                                    break ;                                                
          case	CmdGetVersion   										:	
                                                    #ifdef  _SIM_LCB1_
                                                    TempData.All = 0x07FF ;
                                                    #else
                                                    TempData.All = 0x14F1 ; // for Test
                                                    #endif
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    break ;
          case	CmdGetRpm         									:
                                                    TempData.All = 60 ;
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    break ;
          case	CmdSetRpmGearRatio    							:
          case	CmdSetGenMegPolePair  							:
          case	CmdSetLimitRpmForResis 							:
          case	CmdSetLimitRpmForCharge 						:	     
          case	CmdSetMachineType  									:
          case	CmdSetResistanceTypeAndResistance		:	
          case	CmdSetPowerOff 											:			
          case	CmdSetWatts													:
          case	CmdSetBatteryCharge  								:																													
          case  CmdSetPwm                           :
          case  CmdSetEMagnetCurrent                :
          case  CmdSetBeginBatteryCharge            :
                                                    break ;            
          case	CmdUpdateProgram										:	
                                                    //
                                                    RetData[0] = 'N' ;
                                                    RetDataLength = 1 ;
                                                    TempData.By.High = BootuartRxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = BootuartRxAnalsys.member.Data[1] ; // Low byte
                                                    //	
                                                    if( TempData.All == 0x4A53 ) // "JS"
                                                        {
                                                        if( Boot_CheckLoaderProgram() == _LoaderAPIOk_ )
                                                            {
                                                            RetData[0] = 'Y' ;	
                                                            JustIntoUpdateAPI = _WaitIntoUpdate ;
                                                            }
                                                        }	
                                                    //
                                                    if( TempData.All == 0x4A54 ) // "JT" Update Loader
                                                        {
                                                        RetData[0] = 'Y' ;	
                                                        }	
                                                    //
                                                    break ;                                                
          case	CmdLCBDeviceData										:
                                                    //
                                                    StartAddr.By.LL = BootuartRxAnalsys.member.Data[1] ;
                                                    StartAddr.By.LH = BootuartRxAnalsys.member.Data[2] ;
                                                    StartAddr.By.HL = BootuartRxAnalsys.member.Data[3] ;
                                                    StartAddr.By.HH = BootuartRxAnalsys.member.Data[4] ;
                                               
                                                    //
                                                    RetData[0]=SubCommand ;
                                                    switch( SubCommand )
                                                      {
                                                                                 // R   T
                                                      case GetEEPromMemorySizes :// 3		2
                                                      case GetEEPromMemoryData  ://	3		2
                                                      case GetECBCurrent				:// 3   4
                                                      case GetEStopCapacitance	:// 3   4
                                                      case GetDCBusStatus       :// 3   4
                                                                                break ;
                                                      case GetLCBVersion        :
                                                                                LCBVerInfo.Ver.MCU_A_Loader = 0x1300 ;
                                                                                LCBVerInfo.Ver.MCU_A_API = 0x1300 ;
                                                                                LCBVerInfo.Ver.MCU_B_Loader = 0x13FF ;
                                                                                LCBVerInfo.Ver.MCU_B_API = 0x13FF ;
                                                                                for( RetDataLength = 0 ; RetDataLength < 8 ; RetDataLength++ )
                                                                                    {
                                                                                    RetData[RetDataLength+1] = LCBVerInfo.VerData[RetDataLength] ;
                                                                                    }
                                                                                RetDataLength = 9 ;
                                                                                break ;                                                                                
                                                      case EraseLoaderMemory    :// 1+4+4
                                                                                EndAddr.By.LL = BootuartRxAnalsys.member.Data[5] ;
                                                                                EndAddr.By.LH = BootuartRxAnalsys.member.Data[6] ;
                                                                                EndAddr.By.HL = BootuartRxAnalsys.member.Data[7] ;
                                                                                EndAddr.By.HH = BootuartRxAnalsys.member.Data[8] ;
                                                                                RetData[1] = 2 ;
                                                                                if( StartAddr.All >= LCB_LoaderStartAddr && EndAddr.All <= LCB_LoaderEndAddr )
                                                                                    {
                                                                                    BootFLASH_Unlock() ;
                                                                                    if( BootFLASH_EarseUserProgram(LCB_LoaderMarkStartAddr,LCB_LoaderEndAddr)== FLASH_COMPLETE )
                                                                                        {
                                                                                        if( BootFLASH_EarseUserProgram(StartAddr.All,(EndAddr.All-FLASH_PAGE_SIZE))== FLASH_COMPLETE )
                                                                                            RetData[1] = 0 ;
                                                                                        else
                                                                                            RetData[1] = 1 ;
                                                                                        }
                                                                                    else
                                                                                        RetData[1] = 1 ;
                                                                                    }
                                                                                RetDataLength = 2 ;
                                                                                break ;
                                                      case WriteWordLoaderData  :// 6 + 50
                                                                                WLength = BootuartRxAnalsys.member.Data[5]/2 ;
                                                                                ptr = (unsigned short*)&BootuartRxAnalsys.member.Data[6] ;
                                                                                RetData[1] = 0 ;
                                                                                for( i = 0 ; i < WLength ; i++)
                                                                                    {
                                                                                    if( StartAddr.All >= LCB_LoaderStartAddr && StartAddr.All <= LCB_LoaderEndAddr )
                                                                                        {
                                                                                        if( BootFLASH_ProgramHalfWord((StartAddr.All),*(ptr)) != FLASH_COMPLETE )
                                                                                            {
                                                                                            RetData[1] = 1 ; 
                                                                                            break ;
                                                                                            }
                                                                                        StartAddr.All += 2 ; 
                                                                                        ptr += 1 ;
                                                                                        }
                                                                                    else
                                                                                        {
                                                                                        RetData[1] = 2 ;
                                                                                        break ;
                                                                                        }
                                                                                    }
                                                                                //
                                                                                RetDataLength = 2 ;
                                                                                break ;
                                                      case ReadByteFlashData    :
                                                                                // 6 + 50
                                                                                RetData[1] = 0 ;
                                                                                RetData[2] = BootuartRxAnalsys.member.Data[5] ;
                                                                                cptr = (unsigned char  *)StartAddr.All ;
                                                                                for( WLength = 0 ; WLength < BootuartRxAnalsys.member.Data[5] ; WLength++ )
                                                                                    {
                                                                                    if( (unsigned long)(cptr+WLength) >= 0x08000000 && (unsigned long)(cptr+WLength) <= 0x0801FFFF )
                                                                                        RetData[3+WLength] = *(cptr+WLength) ;
                                                                                    else
                                                                                        {
                                                                                        RetData[1] = 2 ;
                                                                                        RetData[2] = WLength ;
                                                                                        break ;
                                                                                        }
                                                                                    }
                                                                                RetDataLength = 3 + WLength ;
                                                                                break ;
                                                      }
                                                    break ;  
          default																		:
                                                    break ;
          }
      //----------------------------------------------------------------------------
      }
  //----------------------------------------------------------------------------
  // Check Command Error 

  //----------------------------------------------------------------------------
  Bootuart_ReturnData( Command, &RetData[0], RetDataLength ) ;
  //----------------------------------------------------------------------------
  return ;
}




/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char Bootuart_CheckCommandAndLength(unsigned char CheckCmd,unsigned char SubCmd ,unsigned char CheckLen )
{
	unsigned char RetStatus = 2 ;
	switch( CheckCmd )
			{
      // 0
			case 	CmdInitial      										:	
			case 	CmdGetStatus    										:	
			case	CmdGetErrorCode 										:
			case	CmdGetVersion   										:	
			case	CmdSkipErrorCode 										:	
			case	CmdGetRpm         									:	
			case	CmdGetBatteryStatus									:			
      #ifdef INCLINE_MODE  
      case  CmdCalibrate                        :   
      case  CmdGetInclinePercent                :  
      #endif  
																								if( CheckLen == 0 )
																										RetStatus = 1 ;
							
																								break ;
      // 1 byte                                                
			case	CmdSetGenMegPolePair  							:
			case	CmdSetMachineType  									:	
			case	CmdSetPowerOff 											:	
      case  CmdEUPsMode                         :        
      #ifdef INCLINE_MODE    
      case  CmdSetInclineAction                 :  
      #endif  
																								if( CheckLen == 1 )
																										RetStatus = 1 ;
																										
																								break ;		
      // 2 byte                                                
			case	CmdUpdateProgram										:	
			case	CmdSetRpmGearRatio    							:
			case	CmdSetLimitRpmForCharge 						:	
			case	CmdSetLimitRpmForResis 							:
			case	CmdSetResistanceTypeAndResistance		:
			case	CmdSetEMagnetCurrent								:	
      case  CmdSetWatts                         :   
      case  CmdSetPwm                           :
      case  CmdSetBeginBatteryCharge            :
      #ifdef INCLINE_MODE        
      case  CmdSetGapVrCalibrateIncline         :   
      case  CmdSetInclinePercent                :        
      case  CmdSetInclineLocation               :
      case  CmdSetInclineStroke                 :
      #endif        
																								if( CheckLen == 2 )
																										RetStatus = 1 ;	
																											
																								break ;		
      case  CmdLCBDeviceData                    :

                                                switch( SubCmd )
                                                    {
                                                    /*
                                                                               // R   T
                                                    case GetEEPromMemorySizes :// 3		2
                                                    case GetEEPromMemoryData  ://	3		2
                                                    case GetECBCurrent				:// 3   4
                                                    case GetEStopCapacitance	:// 3   4
                                                    case GetDCBusStatus       :// 3   4
                                                                              if( CheckLen == 3 )
																										                              RetStatus = 1 ;	
                                                                              break ;*/
                                                    case EraseLoaderMemory    :// 1+4+4
                                                                              if( CheckLen == 9 )
																										                              RetStatus = 1 ;	
                                                                              break ;
                                                    case WriteWordLoaderData  :// 6 + 50
                                                                              if( CheckLen > 6 && CheckLen < 56 )
																										                              RetStatus = 1 ;	
                                                                              break ;
                                                    case ReadByteFlashData    :
                                                                              if( CheckLen == 6 )
                                                                                  RetStatus = 1 ;
                                                                              break ;
                                                    }
                                                break ;
			default																		:
																								RetStatus = 0 ;
                                                //ErrorCodeStatus.bit.EB0441 = 1 ;
																								break ;
			}	
  //   
	return RetStatus ;
}




//##############################################################################
// PA1 : Output, RS485 DE/RE
void Bootuart_EnableRx(unsigned char Mode)
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




/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void Bootuart_ReturnData( unsigned char TransCmd, unsigned char *DataPtr, unsigned char DataLen )
{
	unsigned short DataLength ;
  unsigned char DataIndex ;
	//
	DataLength = DataMsbLocation + DataLen  ;
  DataIndex = 0 ;
	//
	BootuartTxData.member.Start = LCBStartByte ;
	BootuartTxData.member.Status = 0;//LCBStatus.Full ;
	BootuartTxData.member.Command = TransCmd ;
	BootuartTxData.member.Length = DataLen ;
	//----------------------------------------------------------------------------
  if( DataLen > 0 )
      {
      do
          {
          BootuartTxData.member.Data[DataIndex] = *(DataPtr+DataIndex) ;
          DataIndex += 1 ;
          } while( DataIndex < DataLen ) ;
      }
	//----------------------------------------------------------------------------
	BootuartTxData.member.Data[DataIndex++] = Bootuart_CRC8((unsigned char*)&BootuartTxData.Buffer[0],	DataLength) ;
	//
  BootuartTxDataLength = DataIndex + 4 ;
  BootuartTxDataPoint = 0 ;
  BootuartTxDelayTimeCounter = 0 ;
  //
  //LCBStatus.bit.CommandErrorStatus = 0 ;
	return ;
}


