#ifndef __LCBMAIN_H__
#define __LCBMAIN_H__

#define       _EEPROM_VERSION               0
#define       _PARAMETER_NUMBER             8     


typedef union {
  struct {
    unsigned short Resistance:13 ;
    unsigned short Type:3 ;
  } Member ;
  unsigned short Full ;
} ResistanceDataStruct ;


typedef union {
  struct {
  unsigned char   RecordSize ;                   // 1
  unsigned char   CheckSum ;                     // 1 
  unsigned char   Version ;                      // 1 
  unsigned char   MachineType ;                  // 1
  unsigned char	  PowerOffTime ;                 // 1  
  unsigned char   GenMegPolePair ;               // 1 
  unsigned short  GearRate ;                     // 2 
  ResistanceDataStruct ResistanceType ;          // 2 
  unsigned short  LimitRpmForResistance ;        // 2 
  unsigned short  LimitRpmForCharge ;            // 2 
  unsigned short  ChargeCurrnetForNoResistance ; // 2
  unsigned char   rev[76] ;                      // 76
  } Member ;
  unsigned char   Memory[92] ;
} LCBSystemControlDataStatus ;


typedef struct {
  unsigned short FirstDC12 ;
  unsigned short GeneratorVoltage ;
  unsigned short GeneratorCurrent ;
  unsigned short ElectroMagnetCurrent ;
  //
  unsigned short SecondDC12 ;
  unsigned short BoostVoltage ;  
  unsigned short BoostCurrent ;
  //unsigned short GeneratorCurrent ;
  //unsigned short GeneratorVoltage ;
  unsigned short BatteryChargeCurrnet ;
  unsigned short BatteryVoltage ;
  unsigned short ECBCurrent ;
  unsigned short ECBvoltage ;
  unsigned short ResistanceOffset ;
  //
} LCBADValueStruct ;




typedef struct {
  unsigned short  SetWatts ;
  unsigned short  SetEMCurrent ;
  unsigned short  SetEMPWM ;
  unsigned short  SetInclinePercent ;
  unsigned char   SetInclineManualMode ;
  //
  //unsigned short  BaseWatts ;
  //unsigned short  ConsoleWatts ;
  unsigned short  TragetWatts ;
  unsigned short  GeneratorWatts ;
  unsigned short  ResistanceWatts ;
} UCBDataStruct ;



#define		ERROR_REPORT_DATA_SIZE		20
typedef	union 
{
    struct 
    {
        unsigned char 	Index ;
        unsigned char 	Rev ;
        unsigned short	ErrorCode ;
        unsigned short  EM_Current ;
        unsigned short 	DCBus_Current ;
        unsigned short 	DCBus_Voltage ;
        unsigned short 	Battery_Voltage ;
        unsigned char	LCB_Model ;	 	
        unsigned char   UCB_Model ;
        unsigned char 	LCB_ACIn ;
        unsigned char 	Rev1[4] ;
        unsigned char	CheckSum ;
    } Para ;
    unsigned char All[ERROR_REPORT_DATA_SIZE] ;
} ErrorReportDataType ;

typedef union {
	struct {
		unsigned long 	AccTime ;
		unsigned short 	Value ;
		unsigned char 	CheckSum ;
                unsigned char   rev ;
	} Para ;
	unsigned char All[8] ;
} MaxReportDataType ;

extern 	ErrorReportDataType			ErrorReportData ;
extern 	MaxReportDataType				GeVMaxReportData ;
extern 	MaxReportDataType				GeCMaxReportData ;
extern 	MaxReportDataType				BatCMaxReportData ;
extern 	MaxReportDataType				EMCMaxReportData ;


typedef	struct {
	unsigned char LastIndex ;
	unsigned char LastAddress ;
	unsigned char Index[35] ;
	unsigned char Address[35] ;
	unsigned char Count[35] ;
} LogFileDataType ;

extern LogFileDataType	LogFile ;		

typedef union {
  struct {
    unsigned char SaveParameter:1 ;
    unsigned char UpdateLogMax:1 ;
    unsigned char SaveLogMax:1 ;
    unsigned char UpdateLogFile:1 ;
    unsigned char SaveLogFile:1 ;
    unsigned char SaveErrorLog:1 ;
    unsigned char Rev:2 ;
  } B;
  unsigned char All ;
} EEPROMSaveCtrDataType ;

extern volatile EEPROMSaveCtrDataType EepromControl ;




/* Private function prototypes -----------------------------------------------*/
void LCBMain_SetErPStatus(unsigned char SetStatus) ;
unsigned char LCBMain_GetErPStatus(void) ;
unsigned char LCBMain_GetResetOKStatus(void) ;
unsigned short LCBMain_CalculatorResistanceCommand(unsigned short EMCMD) ;
unsigned char LCBMain_GetBatteryCapacity(void) ;
unsigned char LCBMain_GetDCPluginStatus(void) ;
void LCBMain_SetCutOffResistance(void) ;
void LCBMain_SetUpgradeStatus(void) ;
unsigned char LCBMain_GetConsolePowerStatus(void) ;
#endif /* __LCBMAIN_H__*/

extern volatile unsigned short SystemTimeCounter ;
