/******************** (C) COPYRIGHT 2009 Johnson Fitness Inc. ******************
* File Name          : 
* Author             : 
* Version            : V1.0.0
* Date               : 
* Description        : 
* 0-44F       Program Parameter
* 450-4FF     Error Code List
* 500-        Custom Program Data
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "PinDefine.h"
//#include  "EE93CXX.h"
#include  "LCBMain.h"
#include  "EEPROM.h"
#include  "Mx25L1606E.h"
#include  "JHTCOMMAND.H"
#include  "JHTLCBComm.h"
/* Private typedef -----------------------------------------------------------*/
#define	DATA_STRUCTURE_VER_ADDR		0
#define	DATA_STRUCTURE_VER_SIZE		6
#define	DATA_BASE_VER_1			0x0000
#define	DATA_BASE_VER_2			0x0004
#define	DATA_BASE_VER_3			0x0001

#define PARAMETER_START_ADDR            6
#define PARAMETER_SPACE_SIZE            92

#define	ERROR_REPORT_START_ADDR		(PARAMETER_START_ADDR+PARAMETER_SPACE_SIZE)// 6+92=98
#define	ERROR_REPORT_SPACE_SIZE		800
#define	MAX_REPORT_START_ADDR		(ERROR_REPORT_START_ADDR+ERROR_REPORT_SPACE_SIZE)// 98+800=898
#define	GE_V_REPORT_ADDR		MAX_REPORT_START_ADDR
#define	GE_A_REPORT_ADDR		MAX_REPORT_START_ADDR+7
#define	BAT_A_REPORT_ADDR		MAX_REPORT_START_ADDR+14
#define	EM_A_REPORT_ADDR		MAX_REPORT_START_ADDR+21
#define	MAX_REPORT_SPACE_SIZE		140
#define	TRACKING_REPORT_START_ADDR	(ERROR_REPORT_START_ADDR+ERROR_REPORT_SPACE_SIZE+MAX_REPORT_SPACE_SIZE)// 98+800+140=1038
#define	TRACKING_REPORT_SPACE_SIZE	1010
#define	TRACKING_PACKET_SIZE		7
#define	TRACKING_PACKET_INDEX_MIN	1
#define	TRACKING_PACKET_INDEX_MAX	(TRACKING_REPORT_SPACE_SIZE/TRACKING_PACKET_SIZE)// 1010/7=144

#define	DATA_BASE_TOTAL_SIZE		(ERROR_REPORT_SPACE_SIZE+MAX_REPORT_SPACE_SIZE+TRACKING_REPORT_SPACE_SIZE)// 800+140+1010=1950

#define	ERROR_REPORT_INDEX_MIN		1
#define	ERROR_REPORT_INDEX_MAX		(ERROR_REPORT_SPACE_SIZE/ERROR_REPORT_DATA_SIZE) // 800/20=40

#define	MIN_INDEX_NUMBER		1
#define	MAX_INDEX_NUMBER		254
#define	PARA_MIN_NUMBER			1
#define	PARA_MAX_NUMBER	                35

// 2014.04.09
// A 區 : 格式版本 6 Byte / LCB控制參數資料 92 Byte / 參數最大值資料 140 Byte (參數最大值資料目前僅用28Byte)
#define   _Sector0_Block0_  0
#define   _Sector0_Block1_  1024
#define   _Sector0_Block2_  1024*2 // 2048
#define   _Sector0_Block3_  1024*3 // 3072
#define   _Sector1_Block0_  1024*4 // 4096
#define   _Sector1_Block1_  1024*5 // 5120
#define   _Sector1_Block2_  1024*6 // 6144
#define   _Sector1_Block3_  1024*7 // 7168

#define   _NumberOfBlock_     8
#define   _Sector_BlockSize_  126 //1024 // A區資料=92+7+7+7+7+6
#define   _BlockIndex_        0xAA51
#define   _Sector0_StartNo_   1
#define   _Sector1_StartNo_   5

#define   _Empty_           0xAA
#define   _NoEmpty_         0x55
#define   _WriteError_      0xAA
#define   _WriteOK_         0x55

#define EE_BlockMask        0
#define EE_CheckSum         2
#define EE_ParameterNo      4
#define EE_BlockIDSize      6
#define EE_DataStart        6

// 2014.04.21
// A 區 : 格式版本 6 Byte / LCB控制參數資料 92 Byte / 參數最大值資料 140 Byte 
//#define A_Sector0_ 0
//#define A_Sector1_ 4096
// B 區 : Error Code資料 (Use 800 Byte)
#define B_Sector0_ 4096*2
#define B_Sector1_ 4096*3 
// C 區 : 常用資料 (Use 1010 Byte)
#define C_Sector0_ 4096*4
#define C_Sector1_ 4096*5 
//
#define _EEMemoryBlockSize_ 2048

// C 區塊目前所使用的位址 (Block 0 or Block 1)
unsigned short C_SectorAddress ;
// B 區塊目前所使用的位址 (Block 0 or Block 1)
unsigned short B_SectorAddress ;









// Must be match 4 byte
union {
  struct {
    // for Check Memory  //==> x 表示為以4個byte為1單位
    unsigned short CheckNo ;                      // 2        // save number FF ==> NULL 0xAA51-0xAA54
    unsigned short CheckSum ;                     // 2  x     // 0xFFFF - Total sum = XXXX
    unsigned short MLength ;                      // 2   
    // 開始規劃儲存起始
    unsigned char PARAMETER_SPACE[92] ;           // 92  23x
    unsigned char GE_V_REPORT[7] ;                // 7
    unsigned char GE_A_REPORT[7] ;                // 7
    unsigned char BAT_A_REPORT[7] ;               // 7
    unsigned char EM_A_REPORT[7] ;                // 7  7x
    unsigned char ERROR_REPORT_SPACE[800] ;       // 800  200x
    unsigned char TRACKING_REPORT_SPACE[1010];    // 1010  252x + 2byte    ==> 多的2byte + MLength的2byte可以湊成1組
    // 
  } Parameter ;
  unsigned char Block[_EEMemoryBlockSize_] ;//_Sector_BlockSize_
} EEMemory ;

unsigned char NowMemoryBlockNo ;
unsigned char MemoryStatus ;     // 0xAA = empty , 0x55 = OK
unsigned char MemoryErrorStatus ;
unsigned char _NeedSave = 0;
unsigned char EEPROM_WriteParameter(void);

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ErrorReportDataType	ErrorReportData ;
MaxReportDataType	GeVMaxReportData ;
MaxReportDataType	GeCMaxReportData ;
MaxReportDataType	BatCMaxReportData ;
MaxReportDataType	EMCMaxReportData ;
LogFileDataType		LogFile ;				
extern LCBSystemControlDataStatus  LCBEeprom ;

/* Private function prototypes -----------------------------------------------*/
// 
unsigned char EEPROM_CRC8( unsigned char *ptr, unsigned short DATALENGTH ) ;
//

unsigned short EEPROM_CalculatorChecksum( unsigned char *Dptr, unsigned short size ) ;
unsigned char EEPROM_CheckEEMemoryDataStatus( unsigned long Maddr, unsigned short size, unsigned short CheckData );
unsigned char EEPROM_CheckSectorErase( unsigned long SetcorAddr ) ;
unsigned char EEPROM_VerifyData( unsigned long WriteAddress, unsigned char *Dptr, unsigned short size ) ;
unsigned char EEPROM_GetMemoryErrorStatus(void) ;
void EEPROM_ParameterDefault(void);
//
extern const unsigned char JHTCrcTab[16] ;
//


/*******************************************************************************
* Function Name  : EEPROM_Initial()
* Description    : EEPROM重置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char EEPROM_Initial(void) 
{
  unsigned short CheckData ;
  unsigned short CheckLength ;
  unsigned short CheckSum ;
  unsigned long NowAddress ;
  unsigned char i ;
  //
  CS_EErom(HIGH);
  SK_EErom(LOW);
  DI_EErom(HIGH);
  //
  MemoryErrorStatus = _WriteOK_ ;
  NowMemoryBlockNo = 0 ;
  MemoryStatus = _Empty_ ; 
  // Sector 0
  // Check Block number 0~3
  for( i = 0 ; i < _NumberOfBlock_ ;i++ )
      {
      // Calculator Memory Address  
      if( i != 0 )
          NowAddress = i * _Sector0_Block1_  ;
      else
          NowAddress = _Sector0_Block0_ ;
      //------------------------------------------------------------------------
      CheckData = Mx25L1606E_ReadWord(NowAddress) ;
      if( CheckData == (_BlockIndex_+i) )
          {
          CheckSum = Mx25L1606E_ReadWord((NowAddress+EE_CheckSum)) ;
          if( EEPROM_CheckEEMemoryDataStatus((NowAddress+EE_DataStart),(_Sector_BlockSize_-EE_DataStart),CheckSum) == 0 )
              {
              CheckLength = Mx25L1606E_ReadWord(NowAddress+EE_ParameterNo) ;  
              if( CheckLength == EE_ParameterMax )
                  {
                  NowMemoryBlockNo = CheckData - _BlockIndex_ + 1 ;
                  MemoryStatus = _NoEmpty_ ;
                  }
              }
          }
      else
          {
          if(( NowMemoryBlockNo != 0 ) && (CheckData == 0xFFFF ) )
              break ;
          }
      }
  // 2014.04.21
  EEPROM_InitialParameter_B();
  EEPROM_InitialParameter_C();
  //
  
  if( MemoryStatus == _NoEmpty_ )
  {// 取位置
      //  
      if( (NowMemoryBlockNo-1) != 0 )
          NowAddress = (NowMemoryBlockNo-1) * _Sector0_Block1_  ;
      else
          NowAddress = _Sector0_Block0_ ;
      //
      Mx25L1606E_ReadBlock(NowAddress,126,&EEMemory.Block[0]) ;// _Sector_BlockSize_  // 126=6+92+28
      return C_PASS ;
  }
  else
  {// 重置EEPROM
      // Set default
      NowMemoryBlockNo = 0 ;
      EEPROM_ParameterDefault() ;
      //
      if( EEPROM_CheckSectorErase(_Sector0_Block0_) == 1 )
          Mx25L1606E_SectorErase(0) ;
      
      if( EEPROM_CheckSectorErase(_Sector1_Block0_) == 1 )
          Mx25L1606E_SectorErase(1) ;
      //
      MemoryErrorStatus = EEPROM_WriteParameter() ;
      //
  }
  //
  return C_FAIL ;
}

/*******************************************************************************
* Function Name  : EEPROM_WriteInformation()
* Description    : 外掛 EEPROM 資料寫入
* Input          : by_Address = 記憶體儲存位置 , by_Data = 儲存資料
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_WriteInformation(u8 by_Address,u8 by_Data,u8 by_D1,u8 by_D2) 
{
 
  switch( by_Address )
  {
      case EE_PARAMETER_SPACE:  
              EEMemory.Parameter.PARAMETER_SPACE[by_D1] = by_Data ;
              break;   
      case EE_ERROR_REPORT_SPACE:  
              EEMemory.Parameter.ERROR_REPORT_SPACE[by_D1] = by_Data ;
              break;   
      case EE_GE_V_REPORT:  
              EEMemory.Parameter.GE_V_REPORT[by_D1] = by_Data ;
              break;   
      case EE_GE_A_REPORT:  
              EEMemory.Parameter.GE_A_REPORT[by_D1] = by_Data ;
              break;   
      case EE_BAT_A_REPORT:  
              EEMemory.Parameter.BAT_A_REPORT[by_D1] = by_Data ;
              break;   
      case EE_EM_A_REPORT:  
              EEMemory.Parameter.EM_A_REPORT[by_D1] = by_Data ;
              break;   
      case EE_TRACKING_REPORT_SPACE:  
              EEMemory.Parameter.TRACKING_REPORT_SPACE[by_D1] = by_Data ;
              break;   
      default:
              break ;        
  }
  if(by_D2 == 1)
  {
      EEPROM_WriteParameter() ;
  }
  return ;  
}

/*******************************************************************************
* Function Name  : EEPROM_ReadInformation()
* Description    : 外掛 EEPROM 資料讀取
* Input          : by_Address = 記憶體儲存位置
* Output         : None
* Return         : 回傳8bit資料
*******************************************************************************/
unsigned char EEPROM_ReadInformation(unsigned char by_Address,unsigned char by_D)
{
  u8 RetData ;
  //
  switch( by_Address )
  {
      case EE_PARAMETER_SPACE:   
              RetData = EEMemory.Parameter.PARAMETER_SPACE[by_D] ;
              break;   
      case EE_ERROR_REPORT_SPACE:   
              RetData = EEMemory.Parameter.ERROR_REPORT_SPACE[by_D] ;
              break;   
      case EE_GE_V_REPORT:   
              RetData = EEMemory.Parameter.GE_V_REPORT[by_D] ;
              break;   
      case EE_GE_A_REPORT:   
              RetData = EEMemory.Parameter.GE_A_REPORT[by_D] ;
              break;   
      case EE_BAT_A_REPORT:   
              RetData = EEMemory.Parameter.BAT_A_REPORT[by_D] ;
              break;   
      case EE_EM_A_REPORT:   
              RetData = EEMemory.Parameter.EM_A_REPORT[by_D] ;
              break;   
      case EE_TRACKING_REPORT_SPACE:   
              RetData = EEMemory.Parameter.TRACKING_REPORT_SPACE[by_D] ;
              break;  
      case EE_Block:
              RetData = EEMemory.Block[by_D] ;
              break;
      default:
              break ;                  
  }
  return RetData ;
}

unsigned char EEPROM_WriteParameter(void)
{
  unsigned long WriteAddress ;
  unsigned char RetryCount ;
  RetryCount = 0 ;
  if( NowMemoryBlockNo >= 8 )
      NowMemoryBlockNo = 1 ;
  else
      NowMemoryBlockNo += 1 ;
  //
  if( NowMemoryBlockNo == 0 )
      NowMemoryBlockNo = 1 ;
  //
  EEMemory.Parameter.MLength = EE_ParameterMax ;
  EEMemory.Parameter.CheckNo = _BlockIndex_ + (NowMemoryBlockNo-1) ;   
  EEMemory.Parameter.CheckSum = EEPROM_CalculatorChecksum( &EEMemory.Block[EE_DataStart],(_Sector_BlockSize_-EE_DataStart) ) ; 
  do
  {
      switch( NowMemoryBlockNo )
      {
          default  :
                  WriteAddress = _Sector0_Block0_ ;
                  NowMemoryBlockNo = 1 ;
                  //
                  if( EEPROM_CheckSectorErase(WriteAddress) == 1 )
                      Mx25L1606E_SectorErase(0) ;// 一次 Erase 4K byte
                  
                  //
                  break ;
          case 2   :
                  WriteAddress = _Sector0_Block1_ ;
                  break ;
         case 3   :
                  WriteAddress = _Sector0_Block2_ ;
                  break ;
         case 4   :
                  WriteAddress = _Sector0_Block3_ ;
                  break ;
         case 5   :
                  WriteAddress = _Sector1_Block0_ ;
                  //
                  if( EEPROM_CheckSectorErase(WriteAddress) == 1 )
                      Mx25L1606E_SectorErase(1) ;// 一次 Erase 4K byte
                  //
                  break ;
         case 6   :
                  WriteAddress = _Sector1_Block1_ ;
                  break ;
         case 7   :
                  WriteAddress = _Sector1_Block2_ ;
                  break ;
         case 8   :
                  WriteAddress = _Sector1_Block3_ ;
                  break ;
      }
      // 寫入運動相關設定與資訊儲存資料
      Mx25L1606E_WriteBlock((WriteAddress+EE_DataStart),(_Sector_BlockSize_-EE_DataStart),&EEMemory.Block[EE_DataStart]) ;
      // 寫入起始辨識資料
      Mx25L1606E_WriteBlock(WriteAddress,EE_BlockIDSize,&EEMemory.Block[0]) ;   
      //
      if( EEPROM_VerifyData(WriteAddress,&EEMemory.Block[0],_Sector_BlockSize_) == _WriteError_ )
      {
          RetryCount += 1 ;
          if( NowMemoryBlockNo >= 8 )
              NowMemoryBlockNo = 1 ;
          else
              NowMemoryBlockNo += 1 ;
          // for Check Memory
          EEMemory.Parameter.CheckNo = _BlockIndex_+ ( NowMemoryBlockNo-1 );   
          EEMemory.Parameter.CheckSum = EEPROM_CalculatorChecksum( &EEMemory.Block[EE_DataStart],(_Sector_BlockSize_-EE_DataStart) ) ;    
          EEMemory.Parameter.MLength = EE_ParameterMax ;
          //
          if( NowMemoryBlockNo == _Sector1_StartNo_ )
          {
              if( EEPROM_CheckSectorErase(_Sector1_Block0_) == 1 )
                  Mx25L1606E_SectorErase(1) ;
          }
          else
          {
              if( NowMemoryBlockNo == _Sector0_StartNo_ )
              {
                  if( EEPROM_CheckSectorErase(_Sector0_Block0_) == 1 )
                      Mx25L1606E_SectorErase(0) ;
              }
          }
          //
      }
      else
      {
          if( NowMemoryBlockNo > _Sector1_StartNo_ )
          {
              if( EEPROM_CheckSectorErase(_Sector0_Block0_) == 1 )
                  Mx25L1606E_SectorErase(0) ;
          }
          else
          {
              if( NowMemoryBlockNo != _Sector1_StartNo_ )
              {
                  if( NowMemoryBlockNo > _Sector0_StartNo_ )
                  {
                      if( EEPROM_CheckSectorErase(_Sector1_Block0_) == 1 )
                          Mx25L1606E_SectorErase(1) ;
                  }
              }
          }
          return _WriteOK_ ;
      }
  } while( RetryCount < 5 ) ;
  //
  return _WriteError_ ;
}
/*******************************************************************************
* Function Name  : EEPROM_ParameterDefault()
* Description    : 載入預設參數
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/  
void EEPROM_ParameterDefault(void)
{
  unsigned short i ;
  
  for( i = 0 ; i < _EEMemoryBlockSize_ ; i++ )//_Sector_BlockSize_
  {
      EEMemory.Block[i] = 0 ;
  }
  //
  LCBEeprom.Member.RecordSize = _PARAMETER_NUMBER ;
  LCBEeprom.Member.Version = _EEPROM_VERSION ;      
  LCBEeprom.Member.ResistanceType.Full = 0x0087 ; // EM,13.5ohm
  LCBEeprom.Member.LimitRpmForResistance = 25 ; //
  LCBEeprom.Member.LimitRpmForCharge = 25 ; //
  // Fixed Parameter Default
  LCBEeprom.Member.PowerOffTime = 1 ;  
  LCBEeprom.Member.GenMegPolePair = 4 ;// 極數
  LCBEeprom.Member.MachineType = _BikeEP_ ;
  LCBEeprom.Member.GearRate = _DefaultGearRate ; // 速比 840
  LCBEeprom.Member.ChargeCurrnetForNoResistance = 45 ;
  
  EEPROM_SaveParameter(0);// 0:將預設值放入暫存區
  LogFile.LastAddress = 1 ;
  LogFile.LastIndex = 1 ;
  return ;
}

/*******************************************************************************
* Function Name  : EEPROM_SaveParameter
* Description    : 
* Input          : 0=放暫存區 1=放入後儲存
* Output         : None
* Return         : None
*******************************************************************************/

void EEPROM_SaveParameter(char by_D)
{
  unsigned char i;
  
  LCBEeprom.Member.CheckSum = EEPROM_CRC8(&LCBEeprom.Memory[2],(PARAMETER_SPACE_SIZE-2)) ;
  for(i = 0;i < PARAMETER_SPACE_SIZE;i++)
  {
      // 放入位置
      EEMemory.Parameter.PARAMETER_SPACE[i] = LCBEeprom.Memory[i];
  }
  // 儲存
  if(by_D == 1) EEPROM_WriteParameter() ;
  //
}

/*******************************************************************************
* Function Name  : EEPROM_CRC8()
* Description    : 
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
unsigned char EEPROM_CRC8( unsigned char *ptr, unsigned short DATALENGTH )
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
* Function Name  : EEPROM_GetErrorReportIndex()
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char EEPROM_GetErrorReportIndex(void)
{
  unsigned char Index ;
  unsigned char lastIndex ;
  unsigned short Addr ;
  unsigned char TempData ;
  //
  lastIndex = EEMemory.Parameter.ERROR_REPORT_SPACE[0];
  for( Index = 1 ; Index < ERROR_REPORT_INDEX_MAX ;Index++) // 40
  {
      Addr = (unsigned short)(Index * ERROR_REPORT_DATA_SIZE ) ;
      TempData = EEMemory.Parameter.ERROR_REPORT_SPACE[Addr];
      if( TempData >= ERROR_REPORT_INDEX_MIN && TempData <= 240 )
      {
          if( lastIndex < TempData )
          {
              if( (TempData-lastIndex) == 1 )
                  lastIndex = TempData ;
              else
                  break ;		
          }
          else
              break ;		
      }
      else
      {
          break ;
      }		
  }
  if( lastIndex >= 240 )
  {
      lastIndex = 0 ; 
  }
  return lastIndex ;
}

/*******************************************************************************
* Function Name  : EEPROM_CheckSumCalculator()
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char EEPROM_CheckSumCalculator(unsigned char *pData, unsigned char Length)
{
  unsigned char i ;
  unsigned char chksum = 0 ;
  
  for( i = 0 ; i < Length ; i++ )
  {
      chksum += *(pData+i) ;
  }
  return chksum ;
}

/*******************************************************************************
* Function Name  : EEPROM_ErrorReportSave()
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_ErrorReportSave(void)
{
  unsigned char i ;
  unsigned char index ;
  unsigned short addr ;
  //unsigned char cnt;
  //
  index = 1 ;
  if( ErrorReportData.Para.Index > 1 )
  {
      index = ErrorReportData.Para.Index % 40 ;
      if( index == 0 ) index = 40 ;
      if( index == 1 ) 
      {
          // 區塊輪循
          if(B_SectorAddress == B_Sector0_)
          {
              if( EEPROM_CheckSectorErase(B_Sector1_) == 1 )
              {
                  Mx25L1606E_SectorErase(3);// Erase 第3區塊
              }
              B_SectorAddress = B_Sector1_;// 載入新位置
          }
          else
          {
              if( EEPROM_CheckSectorErase(B_Sector0_) == 1 )
              {
                  Mx25L1606E_SectorErase(2);// Erase 第2區塊
              }
              B_SectorAddress = B_Sector0_;
          }
          // 
      }
  }
  //
  addr = (unsigned short)((index-1) * ERROR_REPORT_DATA_SIZE ) ;// 換算出對應儲存位置
  //		
  ErrorReportData.Para.CheckSum = EEPROM_CheckSumCalculator(&ErrorReportData.All[0],(ERROR_REPORT_DATA_SIZE-1)) ;
  for( i = 0 ; i < ERROR_REPORT_DATA_SIZE ; i++ )
  {
      // 放入位置
      EEMemory.Parameter.ERROR_REPORT_SPACE[addr+i] = ErrorReportData.All[i];
  }
  // 儲存
  Mx25L1606E_WriteBlock( addr + B_SectorAddress , ERROR_REPORT_DATA_SIZE ,&EEMemory.Parameter.ERROR_REPORT_SPACE[addr]) ;
  if( EEPROM_VerifyData( addr + B_SectorAddress ,&EEMemory.Parameter.ERROR_REPORT_SPACE[addr],ERROR_REPORT_DATA_SIZE) == _WriteError_ )
  {
      //cnt = 1;
  }
  else
  {
      //cnt = 2;
  }
  //
  ErrorReportData.Para.Index += 1 ;
  if( ErrorReportData.Para.Index > 240 )
  {
      ErrorReportData.Para.Index = 1 ;		
  }
  return ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_ErrorReportInitial(void)
{
  unsigned char i ;

  for( i = 0 ; i < ERROR_REPORT_DATA_SIZE ; i++ )
  {
      ErrorReportData.All[i] = 0 ;
  }
  ErrorReportData.Para.Index = EEPROM_GetErrorReportIndex() + 1 ;
  EEPROM_InitialMaxReport() ;
  EEPROM_InitialRealTimeReportIndex() ;
  for(i = 0;i < PARAMETER_SPACE_SIZE;i++)
  {
      LCBEeprom.Memory[i] = EEMemory.Parameter.PARAMETER_SPACE[i];
  }    
  return ;
}

/*******************************************************************************
* Function Name  : EEPROM_InitialMaxReport
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_InitialMaxReport(void)
{
  unsigned char i ;
  //
  for( i = 0; i < 7 ;i++ )
  {
      GeVMaxReportData.All[i] = 0 ;
      GeCMaxReportData.All[i] = 0 ;
      BatCMaxReportData.All[i] = 0 ;
  }
  
  if( EEPROM_CheckSumCalculator(&EEMemory.Parameter.GE_V_REPORT[0],6) == EEMemory.Parameter.GE_V_REPORT[6] )
  {
      for( i = 0; i < 6 ;i++ )
      {
          GeVMaxReportData.All[i] = EEMemory.Parameter.GE_V_REPORT[i];// Temp[i] ;
      }
  }
  if( EEPROM_CheckSumCalculator(&EEMemory.Parameter.GE_A_REPORT[0],6) == EEMemory.Parameter.GE_A_REPORT[6] )
  {
      for( i = 0; i < 6 ;i++ )
      {
          GeCMaxReportData.All[i] = EEMemory.Parameter.GE_A_REPORT[i] ;// Temp[i] ;
      }
  }
  if( EEPROM_CheckSumCalculator(&EEMemory.Parameter.BAT_A_REPORT[0],6) == EEMemory.Parameter.BAT_A_REPORT[6] )
  {
      for( i = 0; i < 6 ;i++ )
      {
          BatCMaxReportData.All[i] = EEMemory.Parameter.BAT_A_REPORT[i] ;//Temp[i] ;
      }
  }						
  return ;
}

/*******************************************************************************
* Function Name  : EEPROM_MaxDataReportSave
* Description    : 分4個區塊作儲存,避免同有區塊掛點
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char EEPROM_MaxDataReportSave(unsigned char Index )
{
  unsigned char Idx ;
  unsigned char i,j ;
  //
  Idx = Index ;
  switch( Idx )
  {
      case 1:
              for( i = 0; i < 6 ;i++ )
              {
                  if( GeVMaxReportData.Para.Value == 0 ) break ;
                  if( GeVMaxReportData.All[i] != EEMemory.Parameter.GE_V_REPORT[i] )//Temp[i]
                  {
                      GeVMaxReportData.All[6] = EEPROM_CheckSumCalculator(&GeVMaxReportData.All[0],6)	;
                      for( j = 0; j < 7 ;j++ )
                      {
                           EEMemory.Parameter.GE_V_REPORT[j] = GeVMaxReportData.All[j] ;// Temp[i] ;
                      }
                      EEPROM_WriteParameter() ;
                      break ;
                  }
              }
              break ;
      case 2:
              for( i = 0; i < 6 ;i++ )
              {
                  if( GeCMaxReportData.Para.Value == 0 ) break ;
                  if( GeCMaxReportData.All[i] != EEMemory.Parameter.GE_A_REPORT[i] )//Temp[i]
                  {
                      GeCMaxReportData.All[6] = EEPROM_CheckSumCalculator(&GeCMaxReportData.All[0],6)	;
                      for( j = 0; j < 7 ;j++ )
                      {
                           EEMemory.Parameter.GE_A_REPORT[j] = GeVMaxReportData.All[j] ;// Temp[i] ;
                      }
                      EEPROM_WriteParameter() ;
                      break ;
                  }
              }
              break ;
      case 3:
              for( i = 0; i < 6 ;i++ )
              {
                  if( BatCMaxReportData.Para.Value == 0 ) break ;
                  if( BatCMaxReportData.All[i] != EEMemory.Parameter.BAT_A_REPORT[i] )//Temp[i]
                  {
                      BatCMaxReportData.All[6] = EEPROM_CheckSumCalculator(&BatCMaxReportData.All[0],6)	;
                      for( j = 0; j < 7 ;j++ )
                      {
                           EEMemory.Parameter.BAT_A_REPORT[j] = GeVMaxReportData.All[j] ;// Temp[i] ;
                      }
                      EEPROM_WriteParameter() ;
                      break ;
                  }
              }
              break ;			
      case 4:
              for( i = 0; i < 6 ;i++ )
              {
                  if( EMCMaxReportData.Para.Value == 0 ) break ;
                  if( EMCMaxReportData.All[i] != EEMemory.Parameter.EM_A_REPORT[i] )//Temp[i]
                  {
                      EMCMaxReportData.All[6] = EEPROM_CheckSumCalculator(&EMCMaxReportData.All[0],6)	;
                      for( j = 0; j < 7 ;j++ )
                      {
                           EEMemory.Parameter.EM_A_REPORT[j] = GeVMaxReportData.All[j] ;// Temp[i] ;
                      }
                      EEPROM_WriteParameter() ;
                      break ;
                  }
              }
              break ;									
      default:
              Idx = 0 ;
              break ;
  }
  Idx += 1 ;
  if( Idx > 4 ) Idx = 1 ;		
  //
  return Idx ;
}

/*******************************************************************************
* Function Name  : EEPROM_InitialRealTimeReportIndex
* Description    : 比對出資料最後要儲存位址
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_InitialRealTimeReportIndex(void)
{
  unsigned short Address ;
  unsigned char i,j ;
  
  union {
    struct{
            unsigned int Data ; // long
            unsigned char Index ;
            unsigned char DataNum ;
            unsigned char Sum ;
            unsigned char rev;
          } Para;
          unsigned char All[8] ;
  } TempData ;
  
  unsigned char TempIndex = 0 ;
  //
  LogFile.LastIndex = 1 ;
  LogFile.LastAddress = 255 ;
  for( i =0 ; i < PARA_MAX_NUMBER ; i++ )
  {
      LogFile.Index[i] = 0 ;
      LogFile.Address[i] = 0 ;
      LogFile.Count[i] = 0 ;
  }
  //
  
  //			
  for( i = 0 ; i < (TRACKING_REPORT_SPACE_SIZE / TRACKING_PACKET_SIZE) ; i++ )// 144
  {
      //Address = TRACKING_REPORT_START_ADDR ;
      Address = ((unsigned short)i * TRACKING_PACKET_SIZE) ;// +=
      
      for(j = 0;j < TRACKING_PACKET_SIZE;j++)
      {// 因TempData資料結構的問題所以需再依據規範重新排序
          if(j == 0)
              TempData.All[4] = EEMemory.Parameter.TRACKING_REPORT_SPACE[j + Address] ;
          else if(j == 1)
              TempData.All[5] = EEMemory.Parameter.TRACKING_REPORT_SPACE[j + Address] ;
          else if(j >= 2 && j <= 5)
              TempData.All[j - 2] = EEMemory.Parameter.TRACKING_REPORT_SPACE[j + Address] ;
          else
              TempData.All[j] = EEMemory.Parameter.TRACKING_REPORT_SPACE[j + Address] ;
      }
      // 2014.04.07
      //Mx25L1606E_ReadBlock( Address + C_SectorAddress, TRACKING_PACKET_SIZE, &TempData.All[0] );
      //EE93CXX_ReadDataFromEeprom(Address,TRACKING_PACKET_SIZE,&TempData.All[0]) ;
      //
      if( EEPROM_CheckSumCalculator(&TempData.All[0],6) == TempData.All[6] )
      {
          if( TempData.Para.Index == 0 || TempData.Para.Index == 0xff )
          {
              if( LogFile.LastAddress == 255 )
              {
                  LogFile.LastAddress = i + 1 ;
              }
          }
          else
          {
          //	
              if( TempIndex == 0 )
              {
                  TempIndex = TempData.Para.Index ;
              }
              else
              {
                  if( TempIndex > TempData.Para.Index )
                  {
                      if( LogFile.LastAddress == 255 )
                      {
                          LogFile.LastAddress = i+1 ;	
                      }
                  }	
                  else
                  {
                      TempIndex = TempData.Para.Index ;		
                  }			
              }									
          //		
              if( TempData.Para.DataNum >= 1 && TempData.Para.DataNum <= PARA_MAX_NUMBER )
              {
                  if( (LogFile.Index[TempData.Para.DataNum-1] == 0) || (LogFile.Index[TempData.Para.DataNum-1] < TempData.Para.Index) )
                  {
                      LogFile.Index[TempData.Para.DataNum-1] = TempData.Para.Index ;
                      LogFile.Address[TempData.Para.DataNum-1] = i+1 ;
                  }
              }
          }
      }
      else
      {
          if( LogFile.LastAddress == 255 )
          {
              LogFile.LastAddress = i + 1 ;
          }
      }		
  }
  //
  if( LogFile.LastAddress == 255 )
  {
      LogFile.LastAddress = 1 ;
  }
  //			
  LogFile.LastIndex = TempIndex+1 ;
  if( LogFile.LastIndex > MAX_INDEX_NUMBER )
  {
      LogFile.LastIndex = 1 ;
  }
  //
  return ;
}


unsigned char EEPROM_CheckSectorErase( unsigned long SetcorAddr )
{
  //
  unsigned short i ;
  for( i = 0 ; i < _SectorBase_ ; i++ )
      {
      if( 0xFF != Mx25L1606E_ReadByte((SetcorAddr+i)) )
          return 1 ;
      }
  //
  return 0 ;
}

//
unsigned char EEPROM_VerifyData( unsigned long WriteAddress, unsigned char *Dptr, unsigned short size ) 
{
  unsigned short i ;
  for( i = 0 ; i < size ; i++ )
      {
      if( *(Dptr+i) != Mx25L1606E_ReadByte((WriteAddress+i)) )
          return _WriteError_ ;
      }
  return _WriteOK_ ;  
}


//------------------------------------------------------------------------------------------------------
unsigned short EEPROM_CalculatorChecksum( unsigned char *Dptr, unsigned short size )
{
  unsigned short CheckSum ;
  unsigned short i ;
  CheckSum = 0 ;
  for( i = 0 ; i < size ; i++ )
      {
      CheckSum += *(Dptr+i) ;
      }
  //
  CheckSum = 0xFFFF - CheckSum ;
  //
  return CheckSum ;
}

//------------------------------------------------------------------------------------------------------
unsigned char EEPROM_CheckEEMemoryDataStatus( unsigned long Maddr, unsigned short size, unsigned short CheckData )
{
  unsigned short CheckSum  ;
  unsigned short i ;
  CheckSum = 0 ;
  for( i = 0; i < size ;i++ )
      {
      CheckSum += Mx25L1606E_ReadByte((Maddr+i)) ;
      }
  //
  if( (CheckSum + CheckData) != 0xFFFF )
      return 1 ;
  //
  return 0 ;
}


//------------------------------------------------------------------------------
unsigned char EEPROM_GetMemoryErrorStatus(void)
{
  if( MemoryErrorStatus == _WriteOK_ )
      {
      return  0 ;
      }
  return 1 ;
}


//------------------------------------------------------------------------------
unsigned char EEPROM_ExportEEMemoryData(unsigned short Addr ,unsigned char *ExportData)
{
  if( Addr >= _Sector_BlockSize_ )
      return 0 ;

  *ExportData = EEMemory.Block[Addr] ;
  return 1 ;
}


//-----------------------------------------------------------------------------------
unsigned char EEPROM_InportEEMemoryData( unsigned short Addr, unsigned char InportData )
{
  if( Addr >= _Sector_BlockSize_ )
      return 0 ;

  EEMemory.Block[Addr] = InportData ;
  return 1 ;
}



















































/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/  
/*
unsigned char EEPROM_WriteByteData( unsigned short addr , unsigned char value )
{
  unsigned char Result = 1 ;
  unsigned char TempData ;
  unsigned char InputData ;
  //----------------------------------------------------------------------------
  InputData = value  ;
  EE93CXX_WriteByteData( addr , InputData ) ;
  EE93CXX_ReadByteData( addr,TempData) ;
  if( TempData != InputData )
      {
      Result = 0 ;
      }
  //----------------------------------------------------------------------------
  return Result ;
}
*/

/*******************************************************************************
* Function Name  : 取消 ?
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
unsigned char EEPROM_WriteWordData( unsigned short addr , unsigned short value )
{
  unsigned char Result = 1 ;
  unsigned char TempData[2] ;
  unsigned char InputData[2] ;
  //----------------------------------------------------------------------------
  InputData[0] = value % 0x100 ;
  InputData[1] = value / 0x100 ;
  // 2014.04.07
  Mx25L1606E_WriteBlock( addr , 2 , InputData ) ;
  Mx25L1606E_ReadBlock( addr , 2 , TempData ) ;
  //EE93CXX_WriteWordData( addr , InputData ) ;
  //EE93CXX_ReadWordData(addr,TempData) ;
  //
  if( ( TempData[0] != InputData[0] ) || ( TempData[1] != InputData[1] )  )
      {
      Result = 0 ;
      }
  //----------------------------------------------------------------------------
  return Result ;
}
*/

/*******************************************************************************
* Function Name  : 取消 ?
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
unsigned char EEPROM_WriteDWordData( unsigned short addr , unsigned long value )
{
  unsigned char Result = 1 ;
  unsigned short StartAddr = addr ;
  // 
  Result = EEPROM_WriteWordData( StartAddr , ( value % 0x10000 )) ;
  if( Result == 1 )
      Result = EEPROM_WriteWordData( StartAddr+2 , ( value / 0x10000 )) ; 
  //
  return Result ;
}
*/





// Save Log data






/*******************************************************************************
* Function Name  : 取消 ?
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
unsigned char EEPROM_CheckDataBaseVersion(void)
{
  unsigned char ReturnStatus ;
  unsigned char TempVer ;//short
  //
  ReturnStatus = 0 ;
  // 2014.04.07
  Mx25L1606E_ReadBlock( DATA_STRUCTURE_VER_ADDR , 2 , &TempVer ) ;
  //EE93CXX_ReadWordData(DATA_STRUCTURE_VER_ADDR,TempVer) ;
  //
  if( TempVer == DATA_BASE_VER_1 )
  {
      // 2014.04.07
      Mx25L1606E_ReadBlock( DATA_STRUCTURE_VER_ADDR+2 , 2 , &TempVer ) ;
      //EE93CXX_ReadWordData(DATA_STRUCTURE_VER_ADDR+2,TempVer) ;
      //
      if( TempVer == DATA_BASE_VER_2 )
      {
          // 2014.04.07
          Mx25L1606E_ReadBlock( DATA_STRUCTURE_VER_ADDR+4 , 2 , &TempVer ) ;
          //EE93CXX_ReadWordData(DATA_STRUCTURE_VER_ADDR+4,TempVer) ;
          //
          if( TempVer == DATA_BASE_VER_3 )
          {
              ReturnStatus = 1 ;	
          }
      }
  }
  //
  if( ReturnStatus == 0 )
  {
      EEPROM_DefaultDataBaseValue() ;
      EEPROM_WriteWordData(DATA_STRUCTURE_VER_ADDR,DATA_BASE_VER_1) ;
      EEPROM_WriteWordData(DATA_STRUCTURE_VER_ADDR+2,DATA_BASE_VER_2) ;
      EEPROM_WriteWordData(DATA_STRUCTURE_VER_ADDR+4,DATA_BASE_VER_3) ;
      
      LogFile.LastAddress = 1 ;
      LogFile.LastIndex = 1 ;
  }			
  //		
  return ReturnStatus ;
}
*/

/*******************************************************************************
* Function Name  : 取消 ?
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void EEPROM_DefaultDataBaseValue(void)
//{
  // 2014.04.07
  //Mx25L1606E_SectorErase(0);
  /*
  unsigned short i ;
  
  EE93CXX_EWEN() ;
  
  for( i = 0 ; i < DATA_BASE_TOTAL_SIZE ; i++ )
                  EE93CXX_EARSE((i+ERROR_REPORT_START_ADDR)) ;
                  
  EE93CXX_EWDS() ;            
          
  */
  //
  //return ;
//}




//------------------------------------------------------------------------------

/*******************************************************************************
* Function Name  : EEPROM_WriteParameter_C
* Description    : C 區塊儲存,一次儲存 7 Byte
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

unsigned char _WatchTest_0,_WatchTest_1;

unsigned char EEPROM_WriteParameter_C(unsigned char NumberIndex)
{
  unsigned short Address ;
  unsigned short BackupAddress ;

  union {
        struct{
              unsigned int Data ;
              unsigned char Index ;
              unsigned char DataNum ;
              unsigned char Sum ;
              unsigned char rev ;
        } Para;
        unsigned char All[8] ;
  } TempData ;
  
  //unsigned char ReadDataBuf[7] ;
  unsigned char cnt ;
  //
  TempData.Para.Data = 0 ;
  BackupAddress = 0 ;
  // 輪巡
  if( LogFile.LastAddress < 1 || 
      LogFile.LastAddress > (TRACKING_REPORT_SPACE_SIZE / TRACKING_PACKET_SIZE) )
  {
      LogFile.LastAddress = 1 ;
  }
  // 1 ~ 35 筆資料
  if( NumberIndex >= PARA_MIN_NUMBER && NumberIndex <= PARA_MAX_NUMBER )
  {
      if( LogFile.Address[NumberIndex-1] == 0 )
      {
          LogFile.Address[NumberIndex-1] = LogFile.LastAddress ;
      }
      else
      {
          if( LogFile.Address[NumberIndex-1] >= 1  && 
              LogFile.Address[NumberIndex-1] <= (TRACKING_REPORT_SPACE_SIZE / TRACKING_PACKET_SIZE) )// <= 144
          {
              Address = C_SectorAddress ;
              Address += ((unsigned short)(LogFile.Address[NumberIndex-1]-1)*TRACKING_PACKET_SIZE) ;
              // 2014.04.07
              Mx25L1606E_ReadBlock( Address, TRACKING_PACKET_SIZE, &TempData.All[0] );
              //EE93CXX_ReadDataFromEeprom(Address,TRACKING_PACKET_SIZE,&TempData.All[0]) ;
              //
              BackupAddress = LogFile.Address[NumberIndex-1] ;
              LogFile.Address[NumberIndex-1] = LogFile.LastAddress ;
          }
      }			
      if( LogFile.Address[NumberIndex-1] < 1  && 
          LogFile.Address[NumberIndex-1] > (TRACKING_REPORT_SPACE_SIZE / TRACKING_PACKET_SIZE) )
      {
          LogFile.Address[NumberIndex-1] = 1 ;
          LogFile.LastAddress = 2 ;	
      }
      // 計算儲存位址
      //Address = C_SectorAddress ;
      Address = ((unsigned short)(LogFile.Address[NumberIndex-1]-1)*TRACKING_PACKET_SIZE) ;
      //
      TempData.Para.Index = LogFile.LastIndex ;
      TempData.Para.DataNum = NumberIndex ;
      TempData.Para.Data += LogFile.Count[NumberIndex-1] ;
      if( TempData.Para.Data == 0 )
      {// Data Zero and not need save
          LogFile.Address[NumberIndex-1] = BackupAddress ;
          return 2 ; 
      }
      else
      {   // Max Limit
          if( TempData.Para.Data > 0xFFFFFFFE	)
          {
              TempData.Para.Data = 0xFFFFFFFE ;
          }            
          //
          TempData.Para.Sum = EEPROM_CheckSumCalculator(&TempData.All[0],6) ;
          for(cnt = 0;cnt < TRACKING_PACKET_SIZE;cnt++)
          {// 因TempData資料結構的問題所以需再依據規範重新排序
              if(cnt == 0)
                  EEMemory.Parameter.TRACKING_REPORT_SPACE[cnt + Address] = TempData.All[4] ;
              else if(cnt == 1)
                  EEMemory.Parameter.TRACKING_REPORT_SPACE[cnt + Address] = TempData.All[5] ;
              else if(cnt >= 2 && cnt <= 5)
                  EEMemory.Parameter.TRACKING_REPORT_SPACE[cnt + Address] = TempData.All[cnt - 2] ;
              else
                  EEMemory.Parameter.TRACKING_REPORT_SPACE[cnt + Address] = TempData.All[cnt] ;
          }
          //EEPROM_WriteParameter() ;
          // 寫入運動相關設定與資訊儲存資料
          Mx25L1606E_WriteBlock( Address + C_SectorAddress , TRACKING_PACKET_SIZE ,&EEMemory.Parameter.TRACKING_REPORT_SPACE[Address]) ;
          if( EEPROM_VerifyData( Address + C_SectorAddress ,&EEMemory.Parameter.TRACKING_REPORT_SPACE[Address],TRACKING_PACKET_SIZE) == _WriteError_ )
          {
              cnt = 1;
          }
          else
          {
              cnt = 2;
          }
          //
          LogFile.LastAddress += 1 ;
          if( LogFile.LastAddress > (TRACKING_REPORT_SPACE_SIZE / TRACKING_PACKET_SIZE) ) // >144
          {
              LogFile.LastAddress = 1 ;		
              // 區塊輪循
              if(C_SectorAddress == C_Sector0_)
              {
                  if( EEPROM_CheckSectorErase(C_Sector1_) == 1 )
                  {
                      Mx25L1606E_SectorErase(5);// Erase 第5區塊
                  }
                  C_SectorAddress = C_Sector1_;// 載入新位置
              }
              else
              {
                  if( EEPROM_CheckSectorErase(C_Sector0_) == 1 )
                  {
                      Mx25L1606E_SectorErase(4);// 第4區塊
                  }
                  C_SectorAddress = C_Sector0_;
              }
              // 
          }
          //
          LogFile.LastIndex += 1 ;
          if(  LogFile.LastIndex > MAX_INDEX_NUMBER )
          {
              LogFile.LastIndex = MIN_INDEX_NUMBER ;
          }
          LogFile.Count[NumberIndex-1] = 0 ;	
      }
      return 1 ;
  }
  //
  return 0 ;

    
}

/*******************************************************************************
* Function Name  : EEPROM_InitialParameter_C
* Description    : C區塊搜尋並載入資料
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_InitialParameter_C(void)
{
  unsigned char i;//,cnt;;
  unsigned char Sector0=0,Sector1=0;
  
  // 比對區塊資料
  for(i=0;i < TRACKING_PACKET_INDEX_MAX;i++)// 144
  {
      if(Mx25L1606E_ReadByte(C_Sector0_ + (i * TRACKING_PACKET_SIZE)) != 0xFF) // 7
          Sector0++;
      if(Mx25L1606E_ReadByte(C_Sector1_ + (i * TRACKING_PACKET_SIZE)) != 0xFF) // 7
          Sector1++;
  }
  // 從較少資料的區塊進行資料處理
  if(Sector0 > Sector1)
  {
      Mx25L1606E_ReadBlock(C_Sector1_,TRACKING_REPORT_SPACE_SIZE,&EEMemory.Parameter.TRACKING_REPORT_SPACE[0]) ;
      C_SectorAddress = C_Sector1_;
  }
  else
  {
      Mx25L1606E_ReadBlock(C_Sector0_,TRACKING_REPORT_SPACE_SIZE,&EEMemory.Parameter.TRACKING_REPORT_SPACE[0]) ;
      C_SectorAddress = C_Sector0_;
  }
  //
  return ;
}

/*******************************************************************************
* Function Name  : EEPROM_InitialParameter_B
* Description    : B區塊搜尋並載入資料
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EEPROM_InitialParameter_B(void)
{
  unsigned char i;//,cnt;
  unsigned char Sector0=0,Sector1=0;
  
  // 比對區塊資料
  for(i=0;i < ERROR_REPORT_INDEX_MAX;i++)// 40
  {
      if(Mx25L1606E_ReadByte(B_Sector0_ + (i * ERROR_REPORT_DATA_SIZE)) != 0xFF) // 20
          Sector0++;
      if(Mx25L1606E_ReadByte(B_Sector1_ + (i * ERROR_REPORT_DATA_SIZE)) != 0xFF) // 20
          Sector1++;
  }
  // 從較少資料的區塊進行資料處理
  if(Sector0 > Sector1 && Sector1 != 0)
  {
      // 取此區塊的資料 800 byte
      Mx25L1606E_ReadBlock(B_Sector1_,ERROR_REPORT_SPACE_SIZE,&EEMemory.Parameter.ERROR_REPORT_SPACE[0]) ;
      B_SectorAddress = B_Sector1_;
  }
  else
  {
      // 取此區塊的資料 800 byte
      Mx25L1606E_ReadBlock(B_Sector0_,ERROR_REPORT_SPACE_SIZE,&EEMemory.Parameter.ERROR_REPORT_SPACE[0]) ;
      B_SectorAddress = B_Sector0_;
  }
  return ;
}



/******************* (C) COPYRIGHT 2011 Johnson Fitness Inc. ***END OF FILE****/





