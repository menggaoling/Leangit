
//------------------------------------------------------------------------------
// 
//------------------------------------------------------------------------------


// Private Variables
/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "JHTLCBComm.h"
#include  "JHTCOMMAND.H"
#include  "PinDefine.h"
#include  "BootUartCommand.h"
#include  "Errorcode.h"
#include  "RPM.h"
#include  "LCBMain.h"
#include  "Boot.h"
#include  "Loader.h"
//#include  "EE93CXX.h"
#include  "EEPROM.h"
#include  "ADC.h"

#ifdef  DebugMonitor
//#include <stdio.h>
//#include <string.h>
#endif

/* External variables -----------------------------------------------------------*/
extern RPMDataStruct                RPMData ;
extern LCBADValueStruct             LCBADC ;
extern UCBDataStruct                UCBData ;
extern LCBSystemControlDataStatus   LCBEeprom ;

/* Private typedef -----------------------------------------------------------*/
typedef union {
  struct {
    unsigned char Start ;
    unsigned char Status ;
    unsigned char Command ;
    unsigned char Length ;
    unsigned char Data[256] ;
  } member ;
  unsigned char Buffer[260] ;
} TxDataStruct ;

typedef union {
  struct {
    unsigned char Start ;
    unsigned char Address ;
    unsigned char Command ;
    unsigned char Length ;
    unsigned char Data[256] ;
  } member ;
  unsigned char Buffer[260] ;
} RxDataStruct ;

/* Private define ------------------------------------------------------------*/
#define _RxTimeoutTime              500
#define _RxDisconnectTime           1800
#define _CheckErrorTime             200
#define _CheckErrorSwapDelayTime    (3000/_CheckErrorTime)
#define _CheckErrorDelayTime        (10000/_CheckErrorTime)
#define _MaxErrorBufferSize         5
#define _UCBOfflineTime             (60000/_RxDisconnectTime)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const unsigned char JHTCrcTab[16]={0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e};
volatile CommControl  CommControlFlag ;
//------------------------------------------------------------------------------
#define RxSize (1024*2+10)//200
unsigned char RxBuffer[RxSize] ;
unsigned char *RxStartPoint; 
unsigned char *RxEndPoint;
// 20120204 Add Tx FIFO Control
#define TxSize (1024*2+10)//200 
unsigned char TxBuffer[TxSize] ;
unsigned char *TxStartPoint; 
unsigned char *TxEndPoint;
//
RxDataStruct RxAnalsys ;
ErrorStatusDataStruct ErrorCodeStatus ;
// 2013.12.03
FeedBackADCStruct FeedBackAdcData ;
ControlDataStruct SendData ;
//
//------------------------------------------------------------------------------
unsigned short TxDataPoint ;
unsigned short TxDataLength ;
TxDataStruct  TxData ;
volatile unsigned char TxDelayTimeCounter ;
LCBStatusDataStruct   LCBStatus ;
volatile unsigned short RxTimeoutCounter ;
volatile unsigned short RxDisconnectCounter ;
volatile unsigned char  UCBOfflineCounter ;
//------------------------------------------------------------------------------
// Error code
unsigned short	ErrorMessage[3][_MaxErrorBufferSize] ;
unsigned char	  RetErrType ;
unsigned short	RetErrorCode ;
volatile unsigned char	  OldRetErrType ;
volatile unsigned short	OldRetErrorCode ;
unsigned short	CheckErrorCodeTimeCounter  ;
unsigned short  CheckErrorCodeDelayTimeCounter ;
unsigned short  ErrorSendDelayTime ;
unsigned char   GetClassCErr ;
// Rx/Tx by pass 
volatile unsigned char UpdateLoader_A_TxRxChangeTime ;
volatile unsigned char EnableTxRxChangeTime ;
// Version Struct
extern const unsigned short LCB_LoaderVersion ;
extern const unsigned short User_LCBVersion  ;
LCBVersionInfo  LCBVersion ;
// 
/* Private function prototypes -----------------------------------------------*/
unsigned char JHTLCBComm_CRC8( unsigned char *ptr, unsigned short DATALENGTH ) ;
void JHTLCBComm_CommandDecoder(unsigned char ECmd ) ;
unsigned char JHTLCBComm_CheckCommandAndLength(unsigned char CheckCmd, unsigned char SubCmd,unsigned char CheckLen ) ;
void JHTLCBComm_ReturnData( unsigned char TransCmd, unsigned char *DataPtr, unsigned char DataLen ) ;
//
void JHTLCBComm_SaveErrorCode( unsigned short ErrType , unsigned short ErrCode ) ;
unsigned short JHTLCBComm_GetErrorCode(void) ;
void JHTLCBComm_SkipErrorCode(void) ;
//
void RxFunc_Init(void) ;
unsigned char* RxFunc_NextPt(unsigned char *pt) ;
unsigned char RxFunc_IsEmpty(void) ;
unsigned char RxFunc_IsFull(void) ;
unsigned short RxFunc_Length(void) ;
void RxFunc_Putc(unsigned char c) ;
unsigned char RxFunc_Getc(void) ;
// 20120204 Add Tx FIFO Control
void TxFunc_Init(void) ;
unsigned char* TxFunc_NextPt(unsigned char *pt) ;
unsigned char TxFunc_IsEmpty(void) ;
unsigned char TxFunc_IsFull(void) ;
unsigned short TxFunc_Length(void) ;
void TxFunc_Putc(unsigned char c) ;
unsigned char TxFunc_Getc(void) ;

//2017/6/16 add protocol version 
unsigned char	  ProtocolVersionType ;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_Initial(void)
{
  unsigned short i ;
  //
  RxFunc_Init() ;             // Initial Rx Function
  TxFunc_Init() ;             // Initial Tx Function
  TxDataPoint = 0 ;           // Clear Tx Data Point
  TxDelayTimeCounter = 0 ;    // Clear Tx Delay Time
  TxDataLength = 0 ;          // Clear Tx Data Length
  LCBStatus.Full = 0 ;        // Clear All Status Bit
  RxTimeoutCounter = 0 ;      // Clear Rx Timeout Count 
  RxDisconnectCounter = 0 ;   // Clear Disconnect Count 
  UCBOfflineCounter = 0 ;     // Clear Offline Count 
  CommControlFlag.Full = 0 ;  // Clear All Control Flag 
  //
  for( i = 0 ; i < _MaxErrorBufferSize ; i++ )
  {  
      ErrorMessage[0][i] = 0 ;
      ErrorMessage[1][i] = 0 ;
      ErrorMessage[2][i] = 0 ;
  }
  RetErrType = 0 ;
  RetErrorCode = 0 ;
  OldRetErrType = 0 ;
  OldRetErrorCode = 0 ;  
  CheckErrorCodeTimeCounter = 0 ;
  CheckErrorCodeDelayTimeCounter = 0 ;
  ErrorSendDelayTime = _CheckErrorDelayTime ;
  GetClassCErr = 0 ;
  ProtocolVersionType = OldVersion;
  //
  LCBVersion.Ver.MCU_B_Loader = LCB_LoaderVersion ;
  LCBVersion.Ver.MCU_B_API = User_LCBVersion ;
  //
  CommControlFlag.bit.DisableErPOnCmd = 1 ;
  CommControlFlag.bit.DirectorRXD = 1 ;
  return ;
}
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_HW_Initial(void)
{
  USART_InitTypeDef 	USART_InitStructure;
  // Initial RxData ;
  RxFunc_Init() ;
  //----------------------------------------------------------------------------
  USART_InitStructure.USART_BaudRate = 9600 ;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  // Configure USART2 
  USART_Init(USART2, &USART_InitStructure);
  // Enable USART2 Receive interrupt 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  // Enable USART2 Transmit interrupt 
  USART_ITConfig(USART2,USART_IT_TC,ENABLE);
  //
  USART_Cmd(USART2, ENABLE);
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
void JHTLCBComm_TxRxInterrupt(void)
{
  unsigned char RxData ;
  
  //----------------------------------------------------------------------------  
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
  {
      RxData = USART_ReceiveData(USART2);
      /* Clear the USART2 Receive interrupt */
      USART_ClearITPendingBit(USART2, USART_IT_RXNE);
      //
      RxFunc_Putc(RxData) ;
      RxTimeoutCounter = 0 ;
      //
  }
  //---------------------------------------------------------------------------
  //
  if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {
      /* Clear the USART2 transmit interrupt */
      USART_ClearITPendingBit(USART2, USART_IT_TC);
      if( TxDataPoint < TxDataLength )
      {
          USART_SendData(USART2,TxData.Buffer[TxDataPoint]) ;
          TxDataPoint += 1 ;
      }
      else
      {
          // End of Transmit change to Receiver
          TxDataPoint = 0 ;
          TxDataLength = 0 ;
          CommControlFlag.bit.TxAction = 0 ;
          //
          if( CommControlFlag.bit.UCBOnline == 1 && LCBMain_GetErPStatus() == 0 )
              CommControlFlag.bit.RxTimeoutCheckStart = 1 ; // Enable Receive Timeout Counter
          //
          if( CommControlFlag.bit.WaitIntoUpdateAPI == 1 )
          {
              CommControlFlag.bit.WaitIntoUpdateAPI = 0 ;
              CommControlFlag.bit.JustIntoUpdateAPI = 1 ;
          }
          
          if( CommControlFlag.bit.WaitIntoUpdateLoader == 1 )
          {
              CommControlFlag.bit.WaitIntoUpdateLoader = 0 ;
              CommControlFlag.bit.JustIntoUpdateLoader = 1 ;
          }
          
          RS485Rx(RXD) ;       
          // 20120204
          CommControlFlag.bit.DirectorRXD = 1 ;
          //
          if( CommControlFlag.bit.CheckErPCommandReturn == 1 )
          {
              LCBMain_SetErPStatus(1) ;
              CommControlFlag.bit.CheckErPCommandReturn = 0 ;
          }
          //
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
void JHTLCBComm_RxProcess(void)
{
  //
  unsigned char *pt1;
	unsigned short i ;
  unsigned short Length ;
  unsigned char CheckSum ;
  //
  void (*JumpFunc)(void) ;
  unsigned long *Addr ;

  //
  if( CommControlFlag.bit.JustIntoUpdateAPI == 1 )
      {
      CommControlFlag.bit.JustIntoUpdateAPI = 0 ;
      Addr =(unsigned long *)(0x08000000 + _LInfo_LoaderStartAddress_) ;
      JumpFunc = (void(*)(void))(*Addr) ;
      JumpFunc() ;// 跳到對應位置
      }  
  else if( CommControlFlag.bit.JustIntoUpdateLoader == 1 )
      {
      CommControlFlag.bit.JustIntoUpdateLoader = 0 ;
      Addr =(unsigned long *)(0x08000000 + _LInfo_BootMainStartAddress_) ;
      JumpFunc = (void(*)(void))(*Addr) ;
      JumpFunc() ;
      }
      // Error Process
  #ifdef _EnableErrorCode_
      JHTLCBComm_ErrorMessageProcess() ;
  #endif
      //
  if( CommControlFlag.bit.RxEndWait5msReturn == 0 )
      {
      if( CommControlFlag.bit.DirectorRXD == 1 || CommControlFlag.bit.TxAction == 0 )
          {
          if( CommControlFlag.bit.TxAction != 0 )
              CommControlFlag.bit.TxAction = 0 ;
          
          if( CommControlFlag.bit.DirectorRXD == 0 )
              {
              RS485Rx(RXD) ;
              CommControlFlag.bit.DirectorRXD = 1 ;  
              }
          //---------------------------------------------------------------------------
          // Check Receive Buffer
          if(RxFunc_IsEmpty())
              {
              return ;
              }
            
          // Check Start Byte
          pt1 = RxStartPoint;
          if( *pt1 != 0x00 )    // UCB_START_BYTE
              {
              RxFunc_Getc();    // drop StartCode byte	
              return ;
              }
          //
          if( RxFunc_Length() > 3 )
              {
              //----------------------------------------------------------------
              RxAnalsys.member.Start = *pt1 ;
              pt1 = RxFunc_NextPt(pt1) ;
              //
              RxAnalsys.member.Address = *pt1 ;
              if( RxAnalsys.member.Address != 0xFF ) // UCB_ADDRESS_BYTE
              {
                  RxFunc_Getc();    // drop StartCode byte	
                  // 2014.08.25
                  if( RxAnalsys.member.Address != 0x00 )
                  {
                      RxFunc_Getc();    // drop StartCode byte
                  }
                  //
                  return ;
              }
              //
              pt1 = RxFunc_NextPt(pt1) ;
              RxAnalsys.member.Command = *pt1 ;
              pt1 = RxFunc_NextPt(pt1) ;
              RxAnalsys.member.Length = *pt1 ;
              pt1 = RxFunc_NextPt(pt1) ;
              Length = RxAnalsys.member.Length ;
              //------------------------------------------------------------
              // 2014.08.25
              // 20130819 UCB Send data error
              // Delete Command = 0
              if( RxAnalsys.member.Command == 0x00 )
              {
                  RxFunc_Getc();    // drop StartCode byte	
                  RxFunc_Getc();    // drop StartCode byte	
                  return ; 
              }
              else
              {
                  // Check Length
                  if( RxAnalsys.member.Length > 10 )
                  {
                      RxFunc_Getc();    // drop StartCode byte	
                      RxFunc_Getc();    // drop StartCode byte	
                      RxFunc_Getc();    // drop StartCode byte	
                      RxFunc_Getc();    // drop StartCode byte	
                      return ; 
                  }
              }
              //    
              //----------------------------------------------------------------
              if( RxFunc_Length() >= (Length+5))
                  {
                  // Move Data to Buffer  
                  for( i = 0 ; i < Length ; i++ )
                      {
                      RxAnalsys.member.Data[i] = *pt1 ;
                      pt1 = RxFunc_NextPt(pt1) ;
                      }
                  //------------------------------------------------------------
                  Length += 4 ;
                  CheckSum = *pt1 ;
                  if( CheckSum == JHTLCBComm_CRC8( &RxAnalsys.Buffer[0],Length ) )
                      {
                      //--------------------------------------------------------
                      CommControlFlag.bit.RxTimeoutCheckStart = 0 ;
                      CommControlFlag.bit.DisconnectionTimeoutCheckStart = 0 ;                            
                      UCBOfflineCounter = 0  ; // Clear UCB Offline Counter
                      RxTimeoutCounter = 0 ;
                      RxDisconnectCounter = 0 ;
                      CommControlFlag.bit.UCBOnline = 1 ;
                      CommControlFlag.bit.UCBOffline = 0 ;
                      //  2014.08.25
                      if( ErrorCodeStatus.bit.EC04A0 == 1 )
                      {
                          ErrorCodeStatus.bit.EC04A0 = 0 ;
                      }
                      //
                      if( CommControlFlag.bit.DisableErPOnCmd == 0 )
                          CommControlFlag.bit.DisconnectionTimeoutCheckStart = 1 ;
                      
                      //---------------------------------------------------------
                      JHTLCBComm_CommandDecoder( RxAnalsys.member.Command ) ;
                      // Clear buffer
                      for( i = 0 ; i < (Length+1) ; i++ )
                      {
                          RxFunc_Getc();    // drop StartCode byte
                      }
                      //
                      CommControlFlag.bit.RxEndWait5msReturn = 1 ;
                      //--------------------------------------------------------
                      // Set Status LED Blink
                      CommControlFlag.bit.StatusLED = ~CommControlFlag.bit.StatusLED ;
                      //--------------------------------------------------------
                      
                      }
                  else
                      {
                      RxFunc_Getc();    // drop StartCode byte
                      CommControlFlag.bit.RxEndWait5msReturn = 0 ;
                      CommControlFlag.bit.TxAction = 0 ;
                      }
                  }
              }
          }
      }
  else
      {
      if( TxDelayTimeCounter > 4 ) 
          {
          if( CommControlFlag.bit.WaitIntoUpdateAPI_A_Status == 0 )
              {
              CommControlFlag.bit.RxEndWait5msReturn = 0 ;
              CommControlFlag.bit.TxAction = 1 ; // 20130618
              RS485Rx(TXD) ;
              CommControlFlag.bit.DirectorRXD = 0 ;
              TxDelayTimeCounter = 0 ;
              TxDataPoint = 1 ;
              USART_SendData(USART2,TxData.Buffer[0]) ;
              }
          else
              TxDelayTimeCounter = 0 ; // 20130430
          }
      }
  //----------------------------------------------------------------------------
  // Error Process
  #ifdef _EnableErrorCode_
  JHTLCBComm_ErrorMessageProcess() ;
  #endif
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
unsigned char JHTLCBComm_CRC8( unsigned char *ptr, unsigned short DATALENGTH )
{
  unsigned char crchecktemp,crchalf,CHECKDATA=0;
  unsigned short crcheckcnt = 0 ;
  //
  for(crcheckcnt =0; crcheckcnt < DATALENGTH; crcheckcnt ++)
      {
      crchecktemp = *(ptr+crcheckcnt);
      crchalf =(CHECKDATA/16);
      CHECKDATA<<=4;
      CHECKDATA^=JHTCrcTab[crchalf ^( crchecktemp /16)];
      crchalf =(CHECKDATA/16);
      CHECKDATA<<=4;
      CHECKDATA^=JHTCrcTab[crchalf ^( crchecktemp &0x0f)];
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
void JHTLCBComm_CommandDecoder(unsigned char ECmd )
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
  } StartAddr ;
  
  unsigned short WLength ;
  unsigned char  *cptr ;
  unsigned char R_Loop;

  RetDataLength = 0 ;
  RetData[0] = 0 ;
  //
  Command = ECmd ;
  SubCommand = RxAnalsys.member.Data[0] ;
  if( JHTLCBComm_CheckCommandAndLength(Command,SubCommand,RxAnalsys.member.Length) == 1 )
      {
      //
      if( Command != CmdEUPsMode )
          {
          if( CommControlFlag.bit.DisableErPOnCmd == 1 )
              {
              if( LCBMain_GetConsolePowerStatus() == 1 )
                  {
                  CommControlFlag.bit.DisableErPOnCmd = 0 ;
                  }
              }
          }
      //
      switch( Command )
          {
          case 	CmdInitial:      		    
                                                    JHTLCBComm_ClearAllErrorMessage(1) ;
          case 	CmdGetStatus:	
                                                    break ;
          case	CmdGetErrorCode 										:
                                                    TempData.All = JHTLCBComm_GetErrorCode() ;
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    // log Upload Error
                                                    OldRetErrType = RetErrType ;
							                                      OldRetErrorCode = RetErrorCode ;
                                                    //
                                                    break ;
          case	CmdSkipErrorCode 										:	
                                                    JHTLCBComm_SkipErrorCode() ;
                                                    break ;                                                
          case	CmdGetVersion   										:	
                                                    #ifdef  _SIM_LCB1_
                                                    TempData.All = 0x07FF ; // for Test
                                                    #else
                                                    TempData.All = 0xFF00;// 不支援指令0x73所以需要回覆0xFF00 User_LCBVersion ; 
                                                    #endif
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    break ;
          case	CmdGetProtocolVersion   										:	
                                                    TempData.All = LCB_ProtocolVersion ;             
                                                    RetData[0] = TempData.By.High; // High byte
                                                    RetData[1] = TempData.By.Low;
                                                    RetDataLength = 2 ;
                                                    ProtocolVersionType = NewVersion;
                                                    break ;                                                    
          case	CmdGetRpm         									: 
                                                    TempData.All = RPMData.AverageRPM ;
                                                    RetData[0] = TempData.By.High ; // High byte
                                                    RetData[1] = TempData.By.Low ;
                                                    RetDataLength = 2 ;
                                                    break ;  
          case	CmdSetRpmGearRatio    							:
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    #ifdef  _SIM_LCB1_
                                                    LCBEeprom.Member.GearRate = _DefaultGearRate ;//845
                                                    #else
                                                    if( LCBEeprom.Member.GearRate != TempData.All )
                                                        {
                                                        LCBEeprom.Member.GearRate = TempData.All ;
                                                        EepromControl.B.SaveParameter = 1 ;
                                                        }   
                                                    #endif
                                                    break ;
          case	CmdSetGenMegPolePair  							:
                                                    if( LCBEeprom.Member.GenMegPolePair != (RxAnalsys.member.Data[0]&0x3F) )
                                                        {
                                                        LCBEeprom.Member.GenMegPolePair = RxAnalsys.member.Data[0] & 0x3F ;
                                                        }
                                                    break ;
          case	CmdSetLimitRpmForResis 							:
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    LCBEeprom.Member.LimitRpmForResistance = TempData.All ;
                                                    break ;
          case	CmdSetLimitRpmForCharge 						:	     
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    LCBEeprom.Member.LimitRpmForCharge = TempData.All ;
                                                    break ;
          case	CmdSetMachineType  									:
                                                    LCBEeprom.Member.MachineType = RxAnalsys.member.Data[0] ;
                                                    if( EepromControl.B.SaveErrorLog == 0 )
                                                        {
                                                        ErrorReportData.Para.UCB_Model = TempData.All ;
                                                        }
                                                    #ifdef _EnableErrorCode_
                                                    if( LCBEeprom.Member.MachineType != _BikeEP_ )
                                                        {
                                                        ErrorCodeStatus.bit.EC02AB = 1 ;
                                                        }
                                                    LCBEeprom.Member.MachineType = _BikeEP_ ;
                                                    #endif
                                                    break ;
          case	CmdSetResistanceTypeAndResistance		:	
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    LCBEeprom.Member.ResistanceType.Full = TempData.All ;
                                                    #ifdef _EnableErrorCode_
                                                    if( LCBEeprom.Member.ResistanceType.Member.Type != _ElectroMagnet_Device_ )
                                                        {
                                                        ErrorCodeStatus.bit.EC02B4 = 1 ;
                                                        }
                                                    else
                                                        {
                                                        if( LCBEeprom.Member.ResistanceType.Full != TempData.All )
                                                            {
                                                            LCBEeprom.Member.ResistanceType.Full = TempData.All ;
                                                            EepromControl.B.SaveParameter = 1 ;
                                                            }
                                                        }
                                                    #endif                                                    
                                                    break ;
                                              
          case	CmdUpdateProgram										:	
                                                    //
                                                    RetData[0] = 'N' ;
                                                    RetDataLength = 1 ;
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    //	
                                                    if( TempData.All == 0x4A53 ) // "JS"
                                                        {
                                                        if( Boot_CheckLoaderProgram() == _LoaderAPIOk_ )
                                                            {
                                                            RetData[0] = 'Y' ;	
                                                            CommControlFlag.bit.WaitIntoUpdateAPI = 1 ;
                                                            }
                                                        }		
                                                    else if( TempData.All == 0x4A54 ) // "JT"
                                                        {
                                                        RetData[0] = 'Y' ;	
                                                        CommControlFlag.bit.WaitIntoUpdateLoader = 1 ;
                                                        }                                                    
                                                    
                                                    //
                                                    break ;                                                
          case	CmdLCBDeviceData										:
                                                    RetData[0]=SubCommand ;
                                                    switch( SubCommand )
                                                        {
                                                                                   // R   T
                                                        case GetLCBInformation    :// 1   4
                                                                                  if(ProtocolVersionType == NewVersion)
                                                                                  {
                                                                                    RetData[1] = LCB_VersionNumber;
                                                                                    RetData[2] = LCB_Type;
                                                                                    RetData[3] = 0x00;
                                                                                    RetData[4] = LCB_FormalVersion;
                                                                                    RetData[5] = LCB_BetaVersion;
                                                                                    RetDataLength = 6 ;                                                                                    
                                                                                  }
                                                                                  else
                                                                                  {
                                                                                    RetData[1] = LCB_Type;
                                                                                    RetData[2] = LCB_VersionNumber;
                                                                                    RetData[3] = LCB_FormalVersion;
                                                                                    RetData[4] = LCB_BetaVersion;
                                                                                    RetDataLength = 5 ;
                                                                                  }
                                                                                  break;
                                                        case GetEEPromMemorySizes :// 3		2
                                                                                  TempData.All = 0x0800 ;
                                                                                  RetData[1] = TempData.By.High ; // High byte
                                                                                  RetData[2] = TempData.By.Low ;
                                                                                  RetDataLength = 3 ;
                                                                                  break ;
                                                        case GetEEPromMemoryData  ://	3		2
                                                                                  TempData.By.High = RxAnalsys.member.Data[1] ; // High byte
                                                                                  TempData.By.Low = RxAnalsys.member.Data[2] ; // Low byte
                                                                                  for(R_Loop = 0;R_Loop < 16;R_Loop++)
                                                                                  {
                                                                                      RetData[R_Loop + 1] = EEPROM_ReadInformation(EE_Block,TempData.All);
                                                                                  }                                                                                  
                                                                                  // 2014.04.07
                                                                                  //Mx25L1606E_ReadBlock(TempData.All,16,&RetData[1]) ;
                                                                                  //EE93CXX_ReadDataFromEeprom(TempData.All,16,&RetData[1]) ;
                                                                                  //
                                                                                  RetDataLength = 17 ;
                                                                                  break ;
                                                        case GetECBCurrent				:// 3   4
                                                                                  TempData.All = ADC_GetCalculatorValue(_ECBCurrent_);//FeedBackAdcData.ElectroMagnetCurrent ;
                                                                                  RetData[1] = TempData.By.High ; // High byte
                                                                                  RetData[2] = TempData.By.Low ;
                                                                                  TempData.All = ADC_GetADC(_ECBCurrent_);//FeedBackAdcData.ADC_ElectroMagnetCurrent ;
                                                                                  RetData[3] = TempData.By.High ; // High byte
                                                                                  RetData[4] = TempData.By.Low ;
                                                                                  RetDataLength = 5 ;
                                                                                  break ;
                                                        case GetDCBusStatus       :// 3   4
                                                                                  TempData.All = ADC_GetCalculatorValue(_GeneratorVoltage_);//FeedBackAdcData.GeneratorVoltage ;
                                                                                  RetData[1] = TempData.By.High ; // High byte
                                                                                  RetData[2] = TempData.By.Low ;
                                                                                  TempData.All = ADC_GetCalculatorValue(_GeneratorCurrent_);//FeedBackAdcData.GeneratorCurrent ;
                                                                                  RetData[3] = TempData.By.High ; // High byte
                                                                                  RetData[4] = TempData.By.Low ;
                                                                                  RetDataLength = 5 ;
                                                                                  break ;
                                                        case GetLCBVersion        :
                                                                                  for( RetDataLength = 0 ; RetDataLength < 8 ; RetDataLength++ )
                                                                                      {
                                                                                      RetData[RetDataLength+1] = LCBVersion.VerData[RetDataLength] ;
                                                                                      }
                                                                                  RetDataLength = 9 ;
                                                                                  break ;
                                                        case ReadByteFlashData    :
                                                                                  // 6 + 50
                                                                                  ///*
                                                                                  //
                                                                                  StartAddr.By.LL = RxAnalsys.member.Data[1] ;
                                                                                  StartAddr.By.LH = RxAnalsys.member.Data[2] ;
                                                                                  StartAddr.By.HL = RxAnalsys.member.Data[3] ;
                                                                                  StartAddr.By.HH = RxAnalsys.member.Data[4] ;
                                                                                  //
                                                                                  RetData[1] = 0 ;
                                                                                  RetData[2] = RxAnalsys.member.Data[5] ;
                                                                                  cptr = (unsigned char  *)StartAddr.All ;
                                                                                  for( WLength = 0 ; WLength < RxAnalsys.member.Data[5] ; WLength++ )
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
                                                                                  //*/
                                                                                  break ;
                                                        }
                                                    break ;                                                
          case	CmdEUPsMode													:
                                                    RetData[0] = 0 ;
                                                    if( LCBMain_GetDCPluginStatus() == 1 )
                                                        {
                                                        if( RxAnalsys.member.Data[0] == 0xFF )
                                                            {
                                                            if( CommControlFlag.bit.DisableErPOnCmd == 1 )
                                                                RetData[0] = 0 ;  
                                                            else
                                                                {
                                                                CommControlFlag.bit.CheckErPCommandReturn = 1 ;
                                                                RetData[0] = 1 ;
                                                                }
                                                            }
                                                        else
                                                            {
                                                            if( RxAnalsys.member.Data[0] == 0x00 )
                                                                {
                                                                LCBMain_SetErPStatus(0) ;
                                                                RetData[0] = 1 ;
                                                                }
                                                            }
                                                        }
                                                    else 
                                                        ErrorCodeStatus.bit.EB0441 = 1 ;
                                                    RetDataLength = 1 ;
                                                    //
                                                    CommControlFlag.bit.DisableErPOnCmd = 1 ;
                                                    JHTLCBComm_ResetTimeoutStatus() ;
                                                    //
                                                    break ;                                                
          case	CmdGetBatteryStatus									:
                                                    RetData[0] = (unsigned char)LCBADC.GeneratorVoltage ;
																										RetData[1] = (unsigned char)(LCBADC.BatteryVoltage / 10) ;
                                                    RetDataLength = 2 ;
                                                    break ;                                                
          case	CmdSetPowerOff 											:			
                                                    LCBEeprom.Member.PowerOffTime = RxAnalsys.member.Data[0] ;
                                                    break ;                                                
          case	CmdSetWatts													:
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    if( UCBData.SetWatts != (TempData.All*10) )
                                                    {
                                                        UCBData.SetWatts = TempData.All*10 ;
                                                    }
                                                    UCBData.SetEMCurrent = 0 ;
                                                    UCBData.SetEMPWM = 0 ;
                                                    break ;  
          /*                                                    
          case	CmdSetBatteryCharge  								:																													
                                                    break ;  
          */                                                    
          #ifdef INCLINE_MODE                            
          case	CmdCalibrate 												:
                                                    break ;                                                      
          case	CmdSetInclineStroke									:
                                                    break ;                                                      
          case 	CmdSetInclinePercent  							:
                                                    break ;                                                
          case	CmdGetInclinePercent  							:
                                                    //TempData.All = LCBADC.InclinePosition ;
                                                    //RetData[0] = TempData.By.High ; // High byte
                                                    //RetData[1] = TempData.By.Low ;
                                                    //RetDataLength = 2 ;
                                                    break ;                                                
          case	CmdSetInclineAction									:
                                                    switch(RxAnalsys.member.Data[0])
                                                        {
                                                        case  ManualInclineUp   :
                                                                                break ;
                                                        case  ManualInclineDown :
                                                                                break ;
                                                        case  ManualInclineStop :
                                                                                break ;
                                                        default                 :
                                                                                ErrorCodeStatus.bit.EB0442 = 1 ;
                                                                                break ;
                                                        }
                                                    break ;                                                
          case	CmdSetGapVrCalibrateIncline					:
                                                    break ; 
          #endif
          #ifndef  _DisablePWM_CMD_    // 因_SIM_LCB1_未打開所以沒作用                                                
          case  CmdSetPwm                           :                                                   //
                                                
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    if( UCBData.SetEMPWM != TempData.All )
                                                        UCBData.SetEMPWM = TempData.All ;
                                                    //
                                                    break ;
          case  CmdSetEMagnetCurrent                :
                                                    //
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    if( UCBData.SetEMCurrent != (TempData.All/10) )
                                                        UCBData.SetEMCurrent = TempData.All/10 ;
                                                    //
                                                    break ;
          #endif                                                    
          case  CmdSetBeginBatteryCharge            :
                                                    // Set the battery charge current of no resistance
                                                    //
                                                    TempData.By.High = RxAnalsys.member.Data[0] ; // High byte
                                                    TempData.By.Low = RxAnalsys.member.Data[1] ; // Low byte
                                                    LCBEeprom.Member.ChargeCurrnetForNoResistance = TempData.All ;
                                                    break ;
          case  CmdGetBatteryCapacity               :
                                                    RetData[0] = 0 ;	
                                                    RetData[1] = LCBMain_GetBatteryCapacity() ;	
                                                    RetDataLength = 2 ;
                                                    break ;
          default																		:
                                                    break ;
          }
      //----------------------------------------------------------------------------
      }
  //----------------------------------------------------------------------------
  // Check Command Error 
  #ifdef _EnableErrorCode_
  if( ErrorCodeStatus.bit.EB0441 == 1 )
      {
      JHTLCBComm_SaveErrorCode( _CLASS_B_,0x0441 ) ; 
      ErrorCodeStatus.bit.EB0441 = 0 ;
      }
  if( ErrorCodeStatus.bit.EB0442 == 1 )
      {
      JHTLCBComm_SaveErrorCode( _CLASS_B_,0x0442 ) ;  
      ErrorCodeStatus.bit.EB0442 = 0 ;
      }
  #endif
  //----------------------------------------------------------------------------
  JHTLCBComm_ReturnData( Command, &RetData[0], RetDataLength ) ;
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
unsigned char JHTLCBComm_CheckCommandAndLength(unsigned char CheckCmd,unsigned char SubCmd ,unsigned char CheckLen )
{
  unsigned char RetStatus = 2 ;

  switch( CheckCmd )
  {
        // 0
        case CmdInitial            :	
        case CmdGetStatus    	 :	
        case CmdGetErrorCode 	 :
        case CmdGetVersion   	 :	
        case CmdGetProtocolVersion   	 :          
        case CmdSkipErrorCode 	 :	
        case CmdGetRpm         	 :	
        case CmdGetBatteryStatus	 :			
        case CmdGetBatteryCapacity :
    #ifdef INCLINE_MODE  
        case CmdCalibrate          :   
        case CmdGetInclinePercent  :  
    #endif  
                 if( CheckLen == 0 ) RetStatus = 1 ;
                 break ;
        // 1 byte                                                
        case CmdSetGenMegPolePair  :
        case CmdSetMachineType  	 :	
        case CmdSetPowerOff 	 :	
        case CmdEUPsMode           :        
    #ifdef INCLINE_MODE    
        case CmdSetInclineAction   :  
    #endif  
                 if( CheckLen == 1 ) RetStatus = 1 ;
                 break ;		
        // 2 byte                                                
        case CmdUpdateProgram      :	
        case CmdSetRpmGearRatio    :
        case CmdSetLimitRpmForCharge:	
        case CmdSetLimitRpmForResis:
        case CmdSetResistanceTypeAndResistance:
        case CmdSetWatts           :   
    #ifndef  _DisablePWM_CMD_   // 因_SIM_LCB1_未打開所以沒作用
        case CmdSetPwm             :
        case CmdSetEMagnetCurrent	 :	      
    #endif  
        case CmdSetBeginBatteryCharge:
    #ifdef INCLINE_MODE        
        case CmdSetGapVrCalibrateIncline:   
        case CmdSetInclinePercent  :        
        case CmdSetInclineLocation :
        case CmdSetInclineStroke   :
        #endif        
                 if( CheckLen == 2 ) RetStatus = 1 ;	
                 break ;		
        case CmdLCBDeviceData      :
                 switch( SubCmd )
                 {
                                                // R   T
                     case GetEEPromMemorySizes :// 3   2
                     case GetEEPromMemoryData  :// 3   2
                     case GetECBCurrent        :// 3   4
                     case GetDCBusStatus       :// 3   4
                     case GetLCBVersion        :// 3   8
                              if( CheckLen == 3 ) RetStatus = 1 ;	
                              break ;
                     case ReadByteFlashData    :
                              if( CheckLen == 6 ) RetStatus = 1 ;
                              break ;
                     case GetLCBInformation    :// 1   4
                              if( CheckLen == 1 ) RetStatus = 1 ;
                              break ;         
                 }
                 break ;
        default:
                 RetStatus = 0 ;
                 ErrorCodeStatus.bit.EB0441 = 1 ;
                 break ;
  }	
  // 
  if( RetStatus == 2 ) ErrorCodeStatus.bit.EB0442 = 1 ;
  //   
  return RetStatus ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_ReturnData( unsigned char TransCmd, unsigned char *DataPtr, unsigned char DataLen )
{
  unsigned short DataLength ;
  unsigned char DataIndex ;
  //
  DataLength = DataMsbLocation + DataLen  ;
  DataIndex = 0 ;
  //
  #ifndef _EnableErrorCode_
  LCBStatus.bit.McbErrorStatus = 0 ;
  LCBStatus.bit.CommandErrorStatus = 0 ;
  #endif
  if( LCBMain_GetResetOKStatus() == 0 )
  {
      LCBStatus.bit.InitialStatus = 1 ;
  }
  else
  {
      LCBStatus.bit.InitialStatus = 0 ;
  }
  TxData.member.Start = LCBStartByte ;
  TxData.member.Status = LCBStatus.Full ;
  TxData.member.Command = TransCmd ;
  TxData.member.Length = DataLen ;
  //----------------------------------------------------------------------------
  if( DataLen > 0 )
  {
      do
      {
      TxData.member.Data[DataIndex] = *(DataPtr+DataIndex) ;
      DataIndex += 1 ;
      } while( DataIndex < DataLen ) ;
  }
  //----------------------------------------------------------------------------
  TxData.member.Data[DataIndex++] = JHTLCBComm_CRC8((unsigned char*)&TxData.Buffer[0],	DataLength) ;
  //
  TxDataLength = DataIndex + 4 ;
  TxDataPoint = 0 ;
  TxDelayTimeCounter = 0 ;
  //
  LCBStatus.bit.CommandErrorStatus = 0 ;
  return ;
}



/*******************************************************************************
* Function Name  : 
* Description    : Must be call into the 1ms interrupt
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_TransmitData(void)
{
  unsigned short TempErrorCode ;
  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------
  // Transmit Delay time Calculator 
  if( CommControlFlag.bit.RxEndWait5msReturn == 1 )
      {
      TxDelayTimeCounter += 1 ;
      }
  else
      TxDelayTimeCounter = 0 ;
  //----------------------------------------------------------------------------
  // Timeout and Disconnect counter 
  if( LCBMain_GetErPStatus() == 1 || CommControlFlag.bit.UCBOnline == 0 || LCBMain_GetResetOKStatus() == 0 || LCBMain_GetConsolePowerStatus() == 0)  // Into the ErP must be disable check rx timeout
      {
      CommControlFlag.bit.DisconnectionTimeoutCheckStart = 0 ;
      CommControlFlag.bit.RxTimeoutCheckStart = 0 ;
      // 20130617
      RxDisconnectCounter = 0 ;
      UCBOfflineCounter = 0 ; 
      RxTimeoutCounter = 0 ;
      //
      }
  
  // Receive Timeout Calculator
  if( CommControlFlag.bit.RxTimeoutCheckStart == 1 )
      {
      RxTimeoutCounter += 1 ;
      #ifdef _EnableRxTimeoutError_
      // Check Communication Disconnect
          if( RxTimeoutCounter > _RxTimeoutTime )
              {
              RxTimeoutCounter = 0 ;
              ErrorCodeStatus.bit.EB0440 = 1 ;
              }  
      #endif
      }
  else
      {
      RxTimeoutCounter = 0 ;
      }
  
  if( CommControlFlag.bit.DisconnectionTimeoutCheckStart == 1 )
      {
      if( LCBMain_GetConsolePowerStatus() == 1 ) // 20130611
          {
          RxDisconnectCounter += 1 ;
          #ifdef _EnableErrorCode_
          // Check Communication Disconnect
          if( RxDisconnectCounter > _RxDisconnectTime )
              {
              RxDisconnectCounter = 0 ;
              ErrorCodeStatus.bit.EC04A0 = 1 ;
              UCBOfflineCounter += 1 ;
              if( UCBOfflineCounter >= _UCBOfflineTime ) // 1.5* 40 = 60 sec.
                  {
                  UCBOfflineCounter = 0 ; // Modify by Kunlung 20130617 _UCBOfflineTime ;  
                  CommControlFlag.bit.UCBOffline = 1 ;
                  }
              }
          #endif
          }
      // 20130611 if console power is off the disable disconnection check
      else 
          {
          CommControlFlag.bit.DisconnectionTimeoutCheckStart = 0 ;
          RxDisconnectCounter = 0 ;
          UCBOfflineCounter = 0 ; // 20130617
          }
      //
      }
  else
      {
      RxDisconnectCounter = 0 ;
      UCBOfflineCounter = 0 ; // 20130617
      }
   
  //----------------------------------------------------------------------------
  // Communication Status LED
  if( CommControlFlag.bit.StatusLED == 1 )
  {
      STATUSLED4(ON) ;
  }
  else
  {
      STATUSLED4(OFF) ;
  }
  //----------------------------------------------------------------------------
  // Automatic Check Error Code 
  if( LCBStatus.bit.McbErrorStatus == 1 )
  {
      STATUSLED3(ON) ;
      CheckErrorCodeTimeCounter = 0 ;
  }
  else
  {
      STATUSLED3(OFF) ;
      CheckErrorCodeTimeCounter += 1 ;
  }
  //----------------------------------------------------------------------------
  // Check Error Code occur
  if( CheckErrorCodeTimeCounter >= _CheckErrorTime  )
  {
  //------------------------------------------------------------------------
  // Add by Kunlung 20130507  
      TempErrorCode = JHTLCBComm_GetErrorCode() ; 
      if( TempErrorCode != 0 )
      {
          // Add by KunLung 20130507  
          if( OldRetErrorCode != TempErrorCode )
          {
              if( RetErrType ==  _CLASS_C_ )
              {
                  CheckErrorCodeDelayTimeCounter = 0 ;
                  CommControlFlag.bit.DelayToSendErrorCode = 0 ;
                  LCBStatus.bit.McbErrorStatus = 1 ;
              }
              else
              {
                  if( OldRetErrorCode == 0 )
                  {
                      CheckErrorCodeDelayTimeCounter = 0 ;
                      CommControlFlag.bit.DelayToSendErrorCode = 0 ;
                      LCBStatus.bit.McbErrorStatus = 1 ;
                  }
                  else
                  {
                      if( CommControlFlag.bit.DelayToSendErrorCode == 0 )
                      {
                          CommControlFlag.bit.DelayToSendErrorCode = 1 ;
                          ErrorSendDelayTime = _CheckErrorSwapDelayTime ;
                      }
                  }
              }
          }
          else
          {
              if( CommControlFlag.bit.DelayToSendErrorCode == 0 )
              {
                  CommControlFlag.bit.DelayToSendErrorCode = 1 ;
                  ErrorSendDelayTime = _CheckErrorDelayTime ;
              }
          }  
          //
          if( CommControlFlag.bit.DelayToSendErrorCode == 1 )
          {
              CheckErrorCodeDelayTimeCounter += 1 ;
              if( CheckErrorCodeDelayTimeCounter >= ErrorSendDelayTime )
              {
                  CheckErrorCodeDelayTimeCounter = 0 ;
                  LCBStatus.bit.McbErrorStatus = 1 ;
                  CommControlFlag.bit.DelayToSendErrorCode = 0 ;
              }
          }
          //
      }
      else
      {    
          LCBStatus.bit.McbErrorStatus = 0 ;
          CheckErrorCodeDelayTimeCounter = 0 ;
      }
      CheckErrorCodeTimeCounter = 0 ;
  }	
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
void JHTLCBComm_ResetTimeoutStatus(void)
{
  if( CommControlFlag.bit.UCBOnline == 1 )
      CommControlFlag.bit.UCBOnline = 0  ;
  
  if( CommControlFlag.bit.DisconnectionTimeoutCheckStart == 1 )
      {
      CommControlFlag.bit.DisconnectionTimeoutCheckStart = 0 ;
      RxDisconnectCounter = 0 ;
      }
  
  if( CommControlFlag.bit.RxTimeoutCheckStart == 1 )
      {
      CommControlFlag.bit.RxTimeoutCheckStart = 0 ;
      RxTimeoutCounter = 0 ;
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
void JHTLCBComm_SaveErrorCode( unsigned short ErrType , unsigned short ErrCode )
{
  unsigned short ErrIndex ;
  unsigned short TempMsg ;
  unsigned short ErrClassType ;
  unsigned short Status ;
  //
  ErrClassType = ErrType ;
  Status = 0 ;
  //--------------------------------------------------------------------------
  for( ErrIndex = 0 ; ErrIndex < _MaxErrorBufferSize ; ErrIndex++ )
  {
      if( ErrorMessage[ErrClassType][ErrIndex] == 0 )
      {
          ErrorMessage[ErrClassType][ErrIndex] = ErrCode ;
          break ;
      }
      else
      {
          if( ErrorMessage[ErrClassType][ErrIndex] == ErrCode )
          {
              Status = 1 ;
              break ;
          }
      }
  }
  //--------------------------------------------------------------------------
  if( ErrIndex >= _MaxErrorBufferSize && Status == 0 )
  {
      for( ErrIndex = 0 ; ErrIndex < (_MaxErrorBufferSize-1) ; ErrIndex++ )
      {
          TempMsg = ErrorMessage[ErrClassType][ErrIndex+1] ;
          ErrorMessage[ErrClassType][ErrIndex] = TempMsg ;
      }
      ErrorMessage[ErrClassType][4] = ErrCode ;
  }
	//-------------------------------------------------------------------------
  // 20130507 Mask
	// LCBStatus.bit.McbErrorStatus = 1 ;
  // 20130522 add
  if( OldRetErrorCode == 0 || ErrCode == 0x0441 )
  {
      CommControlFlag.bit.DelayToSendErrorCode = 0 ;
      LCBStatus.bit.McbErrorStatus = 1 ;
  }
  //
  if( ErrType == _CLASS_C_ )
  {
      //20121121 Add to set resistance to zero
      SendData.EM_CurrentCommand = 0 ;
      SendData.EM_TestPWM = 0 ;
      UCBData.SetWatts = 0 ;
      UCBData.SetEMCurrent = 0 ;
      UCBData.SetEMPWM = 0 ;
      LCBMain_SetCutOffResistance() ;
      //
      if( Status == 0 )
      {
          ErrorReportData.Para.ErrorCode = ErrCode ;
          ErrorReportData.Para.LCB_Model = _BikeEP_ ;
          if( LCBStatus.bit.ACPluginStatus == 1 )
              ErrorReportData.Para.LCB_ACIn = 1 ;
          else
              ErrorReportData.Para.LCB_ACIn = 0 ;
          ErrorReportData.Para.Battery_Voltage = LCBADC.BatteryVoltage ;
          ErrorReportData.Para.DCBus_Voltage = LCBADC.GeneratorVoltage ;
          ErrorReportData.Para.DCBus_Current = LCBADC.GeneratorCurrent ;
          EEPROM_ErrorReportSave() ;
      }
  }
  //--------------------------------------------------------------------------
  return ;
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned short JHTLCBComm_GetErrorCode(void)
{
  unsigned char i ;
  //--------------------------------------------------------------------------
  if( ErrorMessage[_CLASS_C_][0] != 0 )
  {
      RetErrType = _CLASS_C_ ;
      if( ErrorMessage[_CLASS_C_][GetClassCErr] != 0 )
      {
          if( GetClassCErr < (_MaxErrorBufferSize-1) )
          {
              for( i = 0 ; i < _MaxErrorBufferSize ;i++ )
              {	
                  if( ErrorMessage[_CLASS_C_][GetClassCErr] != 0 )
                  {
                      RetErrorCode = ErrorMessage[_CLASS_C_][GetClassCErr] ;
                      break ;
                  }
                  else
                  {
                      if( GetClassCErr != 0 ) GetClassCErr -= 1 ;
                  }		
              }		
              //
              if( GetClassCErr != (_MaxErrorBufferSize-1) && ErrorMessage[_CLASS_C_][GetClassCErr+1] != 0 )
                              GetClassCErr += 1 ;	
          }
          else
          {
              GetClassCErr = 0 ;		
              RetErrorCode = ErrorMessage[_CLASS_C_][GetClassCErr] ;
          }
      }
  }
  else
  {
      GetClassCErr = 0 ; // add by  20110713
      if( ErrorMessage[_CLASS_B_][0] != 0 )
      {
          RetErrType = _CLASS_B_ ;
          RetErrorCode = ErrorMessage[_CLASS_B_][0] ;
      }
      else
      {
          if( ErrorMessage[_CLASS_A_][0] != 0 )
          {
              RetErrType = _CLASS_A_ ;
              RetErrorCode = ErrorMessage[_CLASS_A_][0] ;
          }
          else
          {
              RetErrType = _NoErrorCode_ ;
              RetErrorCode = 0 ;
          }
      }
  }
  //--------------------------------------------------------------------------
  return RetErrorCode ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_SkipErrorCode(void)
{
  unsigned char	ErrIndex ;
  unsigned short TempMsg ;
  //--------------------------------------------------------------------------
  //
  if( RetErrType !=  _CLASS_C_ ) 
  {
      for( ErrIndex = 0 ; ErrIndex < (_MaxErrorBufferSize-1) ; ErrIndex++ )
      {
          TempMsg = ErrorMessage[RetErrType][ErrIndex+1] ;
          ErrorMessage[RetErrType][ErrIndex] = TempMsg ;
      }
      ErrorMessage[RetErrType][4] = 0 ;
  }
  //
  LCBStatus.bit.McbErrorStatus = 0 ;
  //--------------------------------------------------------------------------
  return ;
}
/*******************************************************************************
* Function Name  : JHTLCBComm_ErrorMessageProcess
* Description    : 錯誤碼對應處理
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_ErrorMessageProcess(void)
{	
  // Class C
  if( ErrorCodeStatus.bit.EC04A0 == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x04A0 ) ; 
  }
  #ifndef   _DisableMachineSetError_
  if( ErrorCodeStatus.bit.EC02AB == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x02AB ) ; 
  }
  
  if( ErrorCodeStatus.bit.EC02B4 == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x02B4 ) ; 
  }
  #endif
  //Battery connector reverse
  //if( ErrorCodeStatus.bit.EC01B4 == 1 )
  //{
  //    JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x01B4 ) ; 
  //}
  
  if( ErrorCodeStatus.bit.EC01AC == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x01AC ) ; 
  }
  
  if( ErrorCodeStatus.bit.EC01AF == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_C_, 0x01AF ) ; 
  }
  //----------------------------------------------------------------------------
  // Class B
  if( ErrorCodeStatus.bit.EB014A == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_B_, 0x014A ) ; 
      ErrorCodeStatus.bit.EB014A = 0;
  }
  
  if( ErrorCodeStatus.bit.EB0248 == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_B_, 0x0248 ) ; 
  }
  
  if( ErrorCodeStatus.bit.EB0247 == 1 )
  {
      JHTLCBComm_SaveErrorCode( _CLASS_B_, 0x0247 ) ; 
  }
  /*
	if( ErrorCodeStatus.bit.EB0441 == 1 )
      {
      JHTLCBComm_SaveErrorCode( _CLASS_B_,0x0441 ) ; 
      ErrorCodeStatus.bit.EB0441 = 0 ;
      }
  if( ErrorCodeStatus.bit.EB0442 == 1 )
      {
      JHTLCBComm_SaveErrorCode( _CLASS_B_,0x0442 ) ;  
      ErrorCodeStatus.bit.EB0442 = 0 ;
      }
  */
	//----------------------------------------------------------------------------
  return ;
}



/*******************************************************************************
* Function Name  : JHTLCBComm_ClearAllErrorMessage
* Description    : 錯誤碼清除
* Input          : ClearB 0=清除暫存區 1=暫存區+指定旗標
* Output         : 
* Return         : 
*******************************************************************************/
void JHTLCBComm_ClearAllErrorMessage(unsigned char ClearB)
{
  unsigned char Idx ;
  for( Idx = 0 ; Idx < _MaxErrorBufferSize ; Idx++ )
      {
      ErrorMessage[_CLASS_A_][Idx] = 0 ;
      ErrorMessage[_CLASS_B_][Idx] = 0 ;
      ErrorMessage[_CLASS_C_][Idx] = 0 ;
      }
  // Clear Status Bit
  ErrorCodeStatus.bit.EC04A0 = 0 ;
  // Modify by Kunlung 20130507
  if( ClearB == 1 )
      {
      //ErrorCodeStatus.bit.EC01B4 = 0 ;
      ErrorCodeStatus.bit.EB014A = 0 ;
      ErrorCodeStatus.bit.EB0248 = 0 ;
      }
  //
  ErrorCodeStatus.bit.EC01AC = 0;
  ErrorCodeStatus.bit.EC01AF = 0;
  
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char JHTLCBComm_GetUCBOfflineStatus(void)   
{
  if( CommControlFlag.bit.UCBOffline == 1 )
      return 1 ;
  return 0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char JHTLCBComm_GetUCBOnlineStatus(void)   
{
  if( CommControlFlag.bit.UCBOnline == 1 )
      return 1 ;
  return 0 ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void RxFunc_Init(void)
{
  RxStartPoint = RxEndPoint = &RxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char* RxFunc_NextPt(unsigned char *pt)
{
  return ((pt - &RxBuffer[0]) < (RxSize-1))?(pt+1):&RxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char RxFunc_IsEmpty(void)
{
  return (RxStartPoint == RxEndPoint)?1:0;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char RxFunc_IsFull(void)
{
  return (RxStartPoint == RxFunc_NextPt(RxEndPoint))?1:0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned short RxFunc_Length(void)
{ 
	if(RxStartPoint <= RxEndPoint ) 
	    return RxEndPoint - RxStartPoint ; 
	else 
	    return RxSize - (RxStartPoint - RxEndPoint); 
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void RxFunc_Putc(unsigned char c)
{ 
	if(RxStartPoint == RxFunc_NextPt(RxEndPoint)) 
	    return ; 
	*RxEndPoint = c; 
	RxEndPoint = RxFunc_NextPt(RxEndPoint) ;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char RxFunc_Getc(void)
{ 
	unsigned char result=0; 
  
	if(RxEndPoint != RxStartPoint)
      { 
      result = *RxStartPoint; 
      RxStartPoint = RxFunc_NextPt(RxStartPoint) ;
	    } 
  
	return result;
} 


// 20120204 Add Tx FIFO Control
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void TxFunc_Init(void)
{
  TxStartPoint = TxEndPoint = &TxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char* TxFunc_NextPt(unsigned char *pt)
{
  return ((pt - &TxBuffer[0]) < (TxSize-1))?(pt+1):&TxBuffer[0] ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char TxFunc_IsEmpty(void)
{
  return (TxStartPoint == TxEndPoint)?1:0;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char TxFunc_IsFull(void)
{
  return (TxStartPoint == TxFunc_NextPt(TxEndPoint))?1:0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned short TxFunc_Length(void)
{ 
	if(TxStartPoint <= TxEndPoint ) 
	    return TxEndPoint - TxStartPoint ; 
	else 
	    return TxSize - (TxStartPoint - TxEndPoint); 
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
void TxFunc_Putc(unsigned char c)
{ 
	if(TxStartPoint == TxFunc_NextPt(TxEndPoint)) 
	    return ; 
	*TxEndPoint = c; 
	TxEndPoint = TxFunc_NextPt(TxEndPoint) ;
} 


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char TxFunc_Getc(void)
{ 
	unsigned char result=0; 
  
	if(TxEndPoint != TxStartPoint)
      { 
      result = *TxStartPoint; 
      TxStartPoint = TxFunc_NextPt(TxStartPoint) ;
	    } 
  
	return result;
} 



#ifdef  DebugMonitor

unsigned char DebugOut( void )
{
  /*
  if( CommControlFlag.bit.TxAction == 0 )
      {
      memset(&TxData.Buffer[0],0,260);// clear
      sprintf((char*)&TxData.Buffer[0],"GV:%05d,GA:%05d,EA:%05d,1DC12V:%05d,PWM:%05d,Freq:%05d\r\n",FeedBackAdcData.GeneratorVoltage\
              ,FeedBackAdcData.GeneratorCurrent,FeedBackAdcData.ADC_ElectroMagnetCurrent,FeedBackAdcData.DC12Voltage,FeedBackAdcData.ErrorCode,FeedBackAdcData.RPM_Freq);
      TxDataLength = strlen((char const*)&TxData.Buffer[0]);//==>計算長度
      RS485Rx(TXD) ;
      CommControlFlag.bit.DirectorRXD = 0 ;
      TxDelayTimeCounter = 0 ;
      TxDataPoint = 1 ;
      USART_SendData(USART2,TxData.Buffer[0]) ;
      CommControlFlag.bit.TxAction = 1 ;
      return 1 ;
      }
  */
  return 0 ;
}
#endif


