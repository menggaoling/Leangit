#ifndef __JHTLCBCOMM_H__
#define __JHTLCBCOMM_H__


typedef union {
  struct {
    unsigned long EC04A0:1 ; //UCB Communiaction Disconnection
    //unsigned long EC01A0:1 ; //Incline motor disconnection
    //unsigned long EC01A1:1 ; //Incline motor calibration fail
    //unsigned long EC01A7:1 ; //Incline motor over current
    unsigned long EC01AC:1 ; //Resistance Over Current
    unsigned long EB014A:1 ; //The battery charge over-current or short-circuit side.
    unsigned long EC01AF:1 ; //Resistance or solenoid circuit.
    //unsigned long EC01B2:1 ; //The operation is fail and the current is none for the 1st incline motor.
    unsigned long EC02AB:1 ; //Machine Type Error
    unsigned long EC02B4:1 ; //Resistance Type Error
    //unsigned long EB0140:1 ; //Incline motor operation fail 
    //unsigned long EB0145:1 ; //In the self - powered system, Incline stop when LCB battery is the lowest and RPM is enough
    unsigned long EB0248:1 ; //Battery disconnection or fail
    unsigned long EB0440:1 ; //Timeout receive packet
    unsigned long EB0441:1 ; //Correct packet but LCB without the function
    unsigned long EB0442:1 ; //the received command code from the console is correct and is supported, but it has less or more data arguments
    //unsigned long EC01B4:1 ; //Battery connector reverse
    unsigned long EB0247:1 ; //LCB Fail
    unsigned long Rev:20 ;
  } bit ;
  unsigned long Full ;
} ErrorStatusDataStruct ;


typedef union {
  struct {
    unsigned char DnInclineStatus:1 ;		// 0	0:Stop,1:Down Action		
    unsigned char UpInclineStatus:1 ;       	// 1    0:Stop,1:Up Action		
    unsigned char MainMotorStatus:1 ;	// 2	0:Stop,1:Action
    unsigned char ACPluginStatus:1 ;	// 3	0:Stop,1:Action
    unsigned char BatteryLowStatus:1 ;	// 4	0:Normal,1:Low Battery
    unsigned char CommandErrorStatus:1 ;	// 5	0:Normal,1:Error
    unsigned char McbErrorStatus:1 ;	// 6	0:Normal,1:Error
    unsigned char InitialStatus:1 ;	// 7	0:Final Initial,1:Initial
  } bit ;
  unsigned char Full ;
} LCBStatusDataStruct ;

typedef union {
  struct {
    unsigned short MCU_A_Loader ;
    unsigned short MCU_A_API ;
    unsigned short MCU_B_Loader ;
    unsigned short MCU_B_API ;
  } Ver ;
  unsigned char VerData[8] ;
} LCBVersionInfo ;


typedef union {
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
  unsigned long Full ;
} CommControl ;

// 2013.12.03
typedef union {
  struct {
    unsigned short LCB_MCU_Fail:1 ;
    unsigned short LCB_PowerOn:1 ;
    unsigned short EM_OverCurrent:1 ;
    unsigned short EM_NoConnection:1 ;
    unsigned short EM_CurrentLoopAction:1 ;
    unsigned short EM_ManualPWM:1 ;
    unsigned short Rev:10 ;
  } B ;
  unsigned short All ;
} MCU_A_Status ;

typedef volatile struct {
  //unsigned short GeneratorVoltage ;
  //unsigned short GeneratorCurrent ;
  //unsigned short ElectroMagnetCurrent ;
  //unsigned short DC12Voltage ;
  MCU_A_Status   Status ;
  //unsigned short ADC_GeneratorVoltage ;
  //unsigned short ADC_GeneratorCurrent ;
  //unsigned short ADC_ElectroMagnetCurrent ;
  //unsigned short ADC_DC12Voltage ;  
} FeedBackADCStruct ;

typedef struct {
  unsigned short EM_CurrentCommand ;
  unsigned short EM_TestPWM ;
  unsigned short EM_MaxCurrent ;
  unsigned short EM_Resistance ;
} ControlDataStruct ;

extern FeedBackADCStruct FeedBackAdcData ;
extern ControlDataStruct SendData ;

//


extern LCBVersionInfo  LCBVersion ;
extern LCBStatusDataStruct   LCBStatus ;
/* Private function prototypes -----------------------------------------------*/
void JHTLCBComm_Initial(void) ;
void JHTLCBComm_HW_Initial(void) ;
void JHTLCBComm_TxRxInterrupt(void) ;
void JHTLCBComm_RxProcess(void) ;
void JHTLCBComm_TransmitData(void) ;
void JHTLCBComm_ResetTimeoutStatus(void) ;
void JHTLCBComm_ErrorMessageProcess(void) ;
void JHTLCBComm_ClearAllErrorMessage( unsigned char ClearB ) ;
unsigned char JHTLCBComm_GetUCBOfflineStatus(void)  ;
unsigned char JHTLCBComm_GetUCBOnlineStatus(void)  ;
unsigned char DebugOut( void ) ;
#endif /* __JHTLCBCOMM_H__*/


