#ifndef __MULTIMCUCOMM_H
#define __MULTIMCUCOMM_H



/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
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



//
typedef volatile struct {
  unsigned short GeneratorVoltage ;
  unsigned short GeneratorCurrent ;
  unsigned short ElectroMagnetCurrent ;
  unsigned short DC12Voltage ;
  MCU_A_Status   Status ;
  unsigned short ADC_GeneratorVoltage ;
  unsigned short ADC_GeneratorCurrent ;
  unsigned short ADC_ElectroMagnetCurrent ;
  unsigned short ADC_DC12Voltage ;  
} FeedBackADCStruct ;


//
typedef struct {
  unsigned short EM_CurrentCommand ;
  unsigned short EM_TestPWM ;
  unsigned short EM_MaxCurrent ;
  unsigned short EM_Resistance ;
} ControlDataStruct ;

/* define ------------------------------------------------------------*/
#define _FirstDC12V_                  0
#define _GeneratorVoltage_            1
#define _GeneratorCurrent_            2
#define _ElectroMagnetCurrent_        3
#define _MCUA_Status_                 4
#define _ADC_FirstDC12V_              5
#define _ADC_GeneratorVoltage_        6
#define _ADC_GeneratorCurrent_        7
#define _ADC_ElectroMagnetCurrent_    8


/* function prototypes -----------------------------------------------*/
void MultiMCUComm_Initial(void) ;
void MultiMCUComm_HW_Initial(void) ;
void MultiMCUComm_TxRxInterrupt(void) ;
void MultiMCUComm_Process(void) ;
unsigned char MultiMCUComm_SendCommand(unsigned char Cmd) ;
unsigned short MultiMCUComm_GetFeedbcakValue(unsigned char Source ) ;
void MultiMCUComm_RxTimeout( unsigned short TimeLimit ) ;
//------------------------------------------------------------------------------
extern FeedBackADCStruct FeedBackAdcData ;
extern ControlDataStruct SendData ;


#endif  /* __MULTIMCUCOMM_H*/