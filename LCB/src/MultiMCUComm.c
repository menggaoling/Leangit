


/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "BootUartCommand.h"
#include  "MultiMCUComm.h"
#include  "PinDefine.h"
#include  "JHTCOMMAND.H"
#include  "LCBMain.h"
#include  "JHTLCBComm.h"

/*
extern volatile union {
  struct{
    unsigned long TxAction:1 ;
    unsigned long RxEndWait5msReturn:1 ;
    unsigned long RxTimeoutCheckStart:1 ;
    unsigned long DisconnectionTimeoutCheckStart:1 ;
    unsigned long StatusLED:1 ;
    unsigned long WaitIntoUpdateAPI:1 ;
    unsigned long JustIntoUpdateAPI:1 ;
    unsigned long WaitIntoUpdateLoader:1 ;
    unsigned long JustIntoUpdateLoader:1 ;
    unsigned long ByPassTxRx:1 ;           // updata MCU-A
    unsigned long WaitIntoUpdateAPI_A:1 ;
    unsigned long WaitIntoUpdateAPI_A_Status:1 ;
    unsigned long JustIntoUpdateAPI_A:1 ;
    unsigned long WaitIntoUpdateLoader_A:1 ;
    unsigned long JustIntoUpdateLoader_A:1 ;  
    unsigned long ChangeDirector:1 ;
    unsigned long DirectorRXD:1 ;
    //
    unsigned long CheckErPCommandReturn:1 ;
    unsigned long UCBOnline:1 ;
    unsigned long UCBOffline:1 ;
    unsigned long DisableErPOnCmd:1 ;
    unsigned long DelayToSendErrorCode:1 ;
    unsigned long rev:7 ;
    unsigned long ByPassDataToLoader:1 ;
    unsigned long ByPassDataTx:1 ;
    unsigned long ByPassDataToUCB:1 ;
  } bit ;
  unsigned short Full ;
} CommControlFlag ;
*/
extern volatile CommControl  CommControlFlag ;




// External Function
extern void JHTLCBComm_ReturnData( unsigned char TransCmd, unsigned char *DataPtr, unsigned char DataLen ) ;
extern void TxFunc_Putc(unsigned char c) ;
extern unsigned char RxFunc_Getc(void) ;

/* External variables -----------------------------------------------------------*/
extern LCBADValueStruct  LCBADC ;

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct {
  volatile union {
    struct {
      unsigned char RxStart:1 ;
      unsigned char RxSFC:1 ;
      unsigned char RxOK:1 ;
      unsigned char TxOK:1 ;
      unsigned char rev:1 ;
      unsigned char GetVersionOK:1 ;
      unsigned char Timeout:1 ;
      unsigned char StatusLED:1 ;
    } bit ;
    unsigned char Full ;
  } Status ;
  unsigned char RxDataLength ;
  volatile unsigned long Timeout ;
  unsigned short CheckSumErrorCount ;
} MultiMCUCommControl ;


unsigned char MultiMCU_TxBuffer[200] ;
unsigned char MultiMCU_TxIndex ;
unsigned char MultiMCU_RXBuffer[200] ;
unsigned char MultiMCU_RxIndex ;
unsigned char UpdataStatus ;
FeedBackADCStruct FeedBackAdcData ;
ControlDataStruct SendData ;
//for Test 
unsigned short SendEMCommandTemp ;


/* Private function prototypes -----------------------------------------------*/
void MultiMCUComm_TransmitData(unsigned char Cmd, unsigned char *DataPtr , unsigned char Length) ;
unsigned char MultiMCUComm_ConvertSpecialData( unsigned char *pData, unsigned char InData ) ;

/* Private functions ---------------------------------------------------------*/
void MultiMCUComm_Initial(void)
{
  //
  FeedBackAdcData.GeneratorVoltage = 0 ;
  FeedBackAdcData.GeneratorCurrent = 0 ;
  FeedBackAdcData.ElectroMagnetCurrent = 0 ;
  FeedBackAdcData.DC12Voltage = 0 ;
  FeedBackAdcData.Status.All = 0 ;
  FeedBackAdcData.ADC_GeneratorVoltage = 0 ;
  FeedBackAdcData.ADC_GeneratorCurrent = 0 ;
  FeedBackAdcData.ADC_ElectroMagnetCurrent = 0 ;
  FeedBackAdcData.ADC_DC12Voltage = 0 ;  
  //
  SendData.EM_CurrentCommand = 0 ;
  SendData.EM_MaxCurrent = 200 ;
  SendData.EM_Resistance = 0 ;
  SendData.EM_TestPWM = 0 ;
  //
  MultiMCUCommControl.RxDataLength = 0 ;
  MultiMCUCommControl.Status.Full = 0 ;
  MultiMCUCommControl.Timeout = 0 ;
  MultiMCUCommControl.CheckSumErrorCount = 0 ;
  MultiMCUCommControl.Status.bit.TxOK = 1 ;
  MultiMCU_RxIndex = 0 ;
  MultiMCU_TxIndex = 0 ;
  UpdataStatus = 0 ;
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void MultiMCUComm_HW_Initial(void)
{
  USART_InitTypeDef 	USART_InitStructure;
  //
  MultiMCUComm_Initial() ;
  //----------------------------------------------------------------------------
	USART_InitStructure.USART_BaudRate = 115200 ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// Configure USART1 
	USART_Init(USART1, &USART_InitStructure);
  // Enable USART1 Receive interrupt 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  // Enable USART1 Transmit interrupt 
  USART_ITConfig(USART1,USART_IT_TC,ENABLE);
  //
  USART_Cmd(USART1, ENABLE);
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
void MultiMCUComm_TxRxInterrupt(void)
{
  unsigned char TempData ;

  //
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
      {
      TempData = USART_ReceiveData(USART1);
      /* Clear the USART2 Receive interrupt */
      USART_ClearITPendingBit(USART1, USART_IT_RXNE);
      if( CommControlFlag.bit.ByPassTxRx == 1 )
          {
          if( CommControlFlag.bit.DirectorRXD == 0 )
              {
              TxFunc_Putc(TempData) ;
              // 20130415 modify
              if( TempData == LCB_END )
                  CommControlFlag.bit.ByPassDataToUCB = 1 ;   
              //
              }                  
          }
      else
          {
          if( MultiMCUCommControl.Status.bit.RxOK == 0 )
              {
              switch(TempData)
                  {
                  case  LCB_START :
                                  MultiMCU_RxIndex = 0 ;
                                  MultiMCUCommControl.Status.bit.RxStart = 1 ;
                                  break ;
                  case  LCB_SCF   :
                                  MultiMCUCommControl.Status.bit.RxSFC = 1 ;
                                  break ;
                  case  LCB_END   :
                                  if( MultiMCUCommControl.Status.bit.RxStart == 1 )
                                      {
                                      MultiMCUCommControl.Status.bit.RxOK = 1 ;
                                      MultiMCUCommControl.Status.bit.RxStart = 0 ;
                                      MultiMCUCommControl.RxDataLength = MultiMCU_RxIndex ;
                                      }                             
                                  break;
                  default         :
                                  if( MultiMCUCommControl.Status.bit.RxSFC == 1 )
                                      {
                                      MultiMCU_RXBuffer[MultiMCU_RxIndex] = TempData + LCB_SCF ;
                                      }
                                  else
                                      {
                                      MultiMCU_RXBuffer[MultiMCU_RxIndex] = TempData ;
                                      }
                                  MultiMCU_RxIndex += 1 ;
                                  MultiMCUCommControl.Status.bit.RxSFC = 0 ;
                                  if( MultiMCU_RxIndex >= 100 )
                                      MultiMCU_RxIndex = 0 ;   
                                  break;
                  }
              }
          }
      //
      }
  //
  if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
      {//==>§PÂ_¬O§_µo°e¤¤Â_
      /* Clear the USART1 transmit interrupt */
      USART_ClearITPendingBit(USART1, USART_IT_TC);
      //
      if( CommControlFlag.bit.ByPassTxRx == 0 )
          {
          if( MultiMCUCommControl.Status.bit.TxOK == 0 )
              {
              USART_SendData(USART1,MultiMCU_TxBuffer[MultiMCU_TxIndex]) ;
              if( MultiMCU_TxBuffer[MultiMCU_TxIndex] != LCB_END )
                  {
                  // Transmit Data 
                  MultiMCU_TxIndex += 1 ;
                  }
              else
                  {
                  MultiMCUCommControl.Status.bit.TxOK = 1 ;
                  MultiMCU_TxIndex = 0 ;
                  }
              }
          }
      else
          {
          // Transmit to MCU-A
          if( CommControlFlag.bit.ChangeDirector == 1 )
              {
              RS485Rx(TXD) ;
              CommControlFlag.bit.DirectorRXD = 0 ;
              CommControlFlag.bit.ChangeDirector = 0 ;
              }
          else
              {
              if( CommControlFlag.bit.JustIntoUpdateAPI_A == 1 )
                  {
                  TempData = RxFunc_Getc() ;
                  if( TempData == LCB_END )
                      CommControlFlag.bit.ChangeDirector = 1 ; 
                  USART_SendData(USART1,TempData) ;
                  }
              }        
          /* 20130415 modify
          if( CommControlFlag.bit.ChangeDirector == 1 )
              {
              RS485Rx(TXD) ;
              CommControlFlag.bit.DirectorRXD = 0 ;
              CommControlFlag.bit.ChangeDirector = 0 ;
              }
          */
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
void MultiMCUComm_Process(void)
{
  unsigned char i ;
  unsigned char CheckSum = 0 ;
  unsigned char *ptr ;
  static unsigned char DelayTime = 0 ; // 20130430
  //unsigned char TmpData ;
  //
  if( CommControlFlag.bit.ByPassTxRx == 1 )
      return ;
  //
  if( MultiMCUCommControl.Status.bit.RxOK == 1 )
      {
      // 
      if( MultiMCU_RXBuffer[0] >= 0xA0 && MultiMCU_RXBuffer[0] <= 0xAF )
          {
          // Check Data 
          CheckSum = MultiMCU_RXBuffer[0] ;  
          for( i = 1 ; i < (MultiMCUCommControl.RxDataLength-1) ; i++ )
              {
              /*
              if( MultiMCU_RXBuffer[i] >= LCB_SCF && MultiMCU_RXBuffer[i] <= LCB_END )
                  {
                  TmpData = MultiMCU_RXBuffer[i] & 0x0F ;
                  CheckSum ^= LCB_SCF ;
                  CheckSum ^= TmpData ;
                  }
              else  */
              CheckSum ^= MultiMCU_RXBuffer[i] ;
              }
          //
          if( CheckSum == MultiMCU_RXBuffer[(MultiMCUCommControl.RxDataLength-1)] )
              {
              //
              switch( MultiMCU_RXBuffer[0] )
                  {
                  case _FeedbackADC_            :
                                                if( MultiMCU_RXBuffer[1] == 18 )
                                                    {
                                                    ptr = (unsigned char*)&FeedBackAdcData ;
                                                    for( i = 0 ; i < MultiMCU_RXBuffer[1]; i++ )
                                                        {
                                                        *(ptr+i) = MultiMCU_RXBuffer[2+i] ;
                                                        }
                                                    
                                                    }
                                                ///*  
                                                LCBADC.FirstDC12 = MultiMCUComm_GetFeedbcakValue(_FirstDC12V_) ;
                                                LCBADC.GeneratorVoltage = MultiMCUComm_GetFeedbcakValue(_GeneratorVoltage_) ;
                                                LCBADC.GeneratorCurrent = MultiMCUComm_GetFeedbcakValue(_GeneratorCurrent_) ;
                                                LCBADC.ElectroMagnetCurrent = MultiMCUComm_GetFeedbcakValue(_ElectroMagnetCurrent_) ;
                                                //*/
                                                break ;
                  case  _FeedbackUpdateStatus_  :    
                                                UpdataStatus = MultiMCU_RXBuffer[2] ;
                                                JHTLCBComm_ReturnData( CmdUpdateProgram, &UpdataStatus, 1 ) ;
                                                CommControlFlag.bit.WaitIntoUpdateAPI_A_Status = 0 ;
                                                //
                                                if( UpdataStatus == 'Y' )
                                                    CommControlFlag.bit.JustIntoUpdateAPI_A = 1 ;
                                                else
                                                    CommControlFlag.bit.JustIntoUpdateAPI_A = 0 ;
                                                //
                                                break ;
                  case  _FeedbackUpdateLoaderStatus_  :    
                                                UpdataStatus = MultiMCU_RXBuffer[2] ;
                                                JHTLCBComm_ReturnData( CmdUpdateProgram, &UpdataStatus, 1 ) ;
                                                CommControlFlag.bit.WaitIntoUpdateAPI_A_Status = 0 ;
                                                //
                                                if( UpdataStatus == 'Y' )
                                                    CommControlFlag.bit.JustIntoUpdateLoader_A = 1 ;
                                                else
                                                    CommControlFlag.bit.JustIntoUpdateLoader_A = 0 ;
                                                //
                                                break ; 
                  case  _FeedbackReadFlashMemoryData_ :
                                                break ;
                  case  _FeedbackReadVersionData_     :
                                                if( MultiMCU_RXBuffer[1] == 4 )
                                                    {
                                                    ptr = (unsigned char*)&LCBVersion.Ver.MCU_A_Loader ;
                                                    for( i = 0 ; i < MultiMCU_RXBuffer[1]; i++ )
                                                        {
                                                        *(ptr+i) = MultiMCU_RXBuffer[2+i] ;
                                                        }  
                                                    MultiMCUCommControl.Status.bit.GetVersionOK = 1 ;
                                                    }
                                                break ;
                  default                       :
                                                break ;
                  }
              //
              MultiMCUCommControl.Timeout = 0 ;
              MultiMCUCommControl.Status.bit.Timeout = 0 ;
              MultiMCUCommControl.Status.bit.StatusLED = ~MultiMCUCommControl.Status.bit.StatusLED ;
              //
              }
          else
              {
              MultiMCUCommControl.CheckSumErrorCount += 1 ;
              }
          }
      MultiMCUCommControl.Status.bit.RxOK = 0 ;
      }
  //
  if( MultiMCUCommControl.Status.bit.StatusLED == 1 )
      {
      STATUSLED3(ON) ;
      }
  else
      {
      STATUSLED3(OFF) ;
      }
  //
  if( MultiMCUCommControl.Status.bit.Timeout == 1 )
      {
      FeedBackAdcData.GeneratorVoltage = 0 ;
      FeedBackAdcData.GeneratorCurrent = 0 ;
      FeedBackAdcData.ElectroMagnetCurrent = 0 ;
      FeedBackAdcData.DC12Voltage = 0 ;
      FeedBackAdcData.Status.All = 0 ;
      FeedBackAdcData.ADC_GeneratorVoltage = 0 ;
      FeedBackAdcData.ADC_GeneratorCurrent = 0 ;
      FeedBackAdcData.ADC_ElectroMagnetCurrent = 0 ;
      FeedBackAdcData.ADC_DC12Voltage = 0 ;  
      LCBADC.FirstDC12 = 0 ;
      LCBADC.GeneratorVoltage = 0 ;
      LCBADC.GeneratorCurrent = 0 ;
      LCBADC.ElectroMagnetCurrent = 0 ;
      MultiMCUCommControl.Status.bit.Timeout = 0 ;
      MultiMCUCommControl.Status.bit.GetVersionOK = 0 ;
      CommControlFlag.bit.WaitIntoUpdateAPI_A_Status = 0 ; // 20130430
      }
  
  // Check Version Get
  // 20130430
  DelayTime += 1 ;
  if( CommControlFlag.bit.WaitIntoUpdateAPI_A == 0 && CommControlFlag.bit.WaitIntoUpdateLoader_A == 0 )
      {
      if( MultiMCUCommControl.Status.bit.GetVersionOK == 0 && CommControlFlag.bit.WaitIntoUpdateAPI_A_Status == 0 )
          {
          //MultiMCUComm_SendCommand(_ReadVersion_) ;
          if( DelayTime > 5 )
              {
              DelayTime = 5 ;
              if( MultiMCUComm_SendCommand(_ReadVersion_) == 1 )
                  DelayTime = 0 ;
              }
          }
      }
  //
  return ;
}
//


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char MultiMCUComm_SendCommand(unsigned char Cmd)
{
  //
  if( CommControlFlag.bit.ByPassTxRx == 1 )
    return 1 ;
  //
  if( MultiMCUCommControl.Status.bit.TxOK == 1 )
      {
      switch(Cmd)
          {
          case  _SetEMControlCurrent_   :
                                        //SendEMCommandTemp = LCBMain_CalculatorResistanceCommand(SendData.EM_CurrentCommand) ;
                                        //MultiMCUComm_TransmitData(_SetEMControlCurrent_,(unsigned char*)&SendEMCommandTemp,2) ;
                                        MultiMCUComm_TransmitData(_SetEMControlCurrent_,(unsigned char*)&SendData.EM_CurrentCommand,2) ;
                                        return 1 ;
          case  _SetDefaultVolue_       :
                                        MultiMCUComm_TransmitData(_SetDefaultVolue_,(unsigned char*)&SendData.EM_MaxCurrent,4) ;
                                        return 1 ;
          case  _TestSetEMPWM_          :
                                        MultiMCUComm_TransmitData(_TestSetEMPWM_,(unsigned char*)&SendData.EM_TestPWM,2) ;
                                        return 1 ;
          case  _JustUpdateAPI_         :
                                        MultiMCUComm_TransmitData(_JustUpdateAPI_,0,0) ;
                                        return 1 ;
          case  _JustUpdateLoaderAPI_   :
                                        MultiMCUComm_TransmitData(_JustUpdateLoaderAPI_,0,0) ;
                                        return 1 ;  
          case  _ReadFlashMemoryData_   :
                                        return 1 ;
          case  _ReadVersion_           :
                                        MultiMCUComm_TransmitData(_ReadVersion_,0,0) ;
                                        return 1 ;
          default                       :
                                        break ;
          }
      }
  return 0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void MultiMCUComm_TransmitData(unsigned char Cmd, unsigned char *DataPtr , unsigned char Length)
{
  unsigned char i ;
  unsigned char Index ;
  MultiMCU_TxBuffer[0] = LCB_START ;
  MultiMCU_TxBuffer[1] = Cmd ;
  MultiMCU_TxBuffer[2] = Length ;
  Index = 3 ;
  for( i = 0 ; i < Length; i++ )
      {
      Index += MultiMCUComm_ConvertSpecialData(&MultiMCU_TxBuffer[Index],*(DataPtr+i)) ;
      }
  //
  MultiMCU_TxBuffer[Index] =  MultiMCU_TxBuffer[1] ;                                         
  for( i = 2 ; i < Index ; i++ ) //(MultiMCU_TxBuffer[2]+3)
      MultiMCU_TxBuffer[Index] ^= MultiMCU_TxBuffer[i] ;
  Index += MultiMCUComm_ConvertSpecialData(&MultiMCU_TxBuffer[Index],MultiMCU_TxBuffer[Index]) ;
  //
  MultiMCU_TxBuffer[Index] = LCB_END ;
  // Start Send
  MultiMCUCommControl.Status.bit.TxOK = 0 ;
  MultiMCU_TxIndex = 1 ;
  USART_SendData(USART1,MultiMCU_TxBuffer[0]) ;
  //
  return ;
}


//##############################################################################
// Function : Convert 0xD1,0xD2,0xD0 data to 0xD0+0x00,0xD0+0x01,0xD0+0x02
// Input : 
//        pData: data save memory address
//        InData: Convert Data
// Return : data count
//##############################################################################
unsigned char MultiMCUComm_ConvertSpecialData( unsigned char *pData, unsigned char InData )
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


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned short MultiMCUComm_GetFeedbcakValue(unsigned char Source )
{
  unsigned short Result ;
  Result = 0 ;
  switch(Source)
      {
      case  _FirstDC12V_              :
                                      Result = FeedBackAdcData.DC12Voltage ;
                                      break ;
      case  _GeneratorVoltage_        :
                                      Result = FeedBackAdcData.GeneratorVoltage ;
                                      break ;
      case  _GeneratorCurrent_        :
                                      Result =FeedBackAdcData.GeneratorCurrent ;
                                      break ;
      case  _ElectroMagnetCurrent_    :
                                      Result = FeedBackAdcData.ElectroMagnetCurrent ;
                                      break ;        
      case  _MCUA_Status_               :
                                      Result = FeedBackAdcData.Status.All ;
                                      break ;
      case  _ADC_FirstDC12V_          :
                                      Result = FeedBackAdcData.ADC_DC12Voltage ;
                                      break ;
      case  _ADC_GeneratorVoltage_    :
                                      Result = FeedBackAdcData.ADC_GeneratorVoltage ;
                                      break ;
      case  _ADC_GeneratorCurrent_    :
                                      Result = FeedBackAdcData.ADC_GeneratorCurrent ;
                                      break ;
      case  _ADC_ElectroMagnetCurrent_:
                                      Result = FeedBackAdcData.ADC_ElectroMagnetCurrent ;
                                      break ;
      }
  return Result ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void MultiMCUComm_RxTimeout( unsigned short TimeLimit )
{
  if( TimeLimit != 0 )
      {
      if( MultiMCUCommControl.Status.bit.Timeout == 0 )
          MultiMCUCommControl.Timeout += 1 ;
      
      if( MultiMCUCommControl.Timeout > TimeLimit )
          {
          MultiMCUCommControl.Timeout = 0 ;
          MultiMCUCommControl.Status.bit.Timeout = 1 ;
          }
      }
  else
      {
      if( MultiMCUCommControl.Timeout != 0 )
          MultiMCUCommControl.Timeout = 0 ;
      
      if( MultiMCUCommControl.Status.bit.Timeout != 0 )
          MultiMCUCommControl.Status.bit.Timeout = 0 ;
      }
  return ;
}




