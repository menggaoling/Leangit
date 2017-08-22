#ifndef __ADC_H__
#define __ADC_H__


typedef struct {
  unsigned short max ;
  unsigned short min ;
  unsigned char  count ;
  unsigned long  sum ;
  unsigned short adverage ;
  unsigned short offsetADC ;
} ADCDataStruct ;


typedef union {
  struct {
    unsigned char SetOffset:1 ;
    unsigned char ADCComplete:1 ;
    unsigned char ADCSampleComplete:1 ;
    unsigned char ADCDataComplete:1 ;
    unsigned char ADCDataSatndby:1 ;
    unsigned char rev:3 ;
  } B ;
  unsigned char All ;
} ADCStatusReg ;

//------------------------------------------------------------------------------
#define   _BatteryVoltage_        0 // Battery voltage
#define   _BatteryCurrent_        1 // Battery Current
#define   _BoostCurrent_          2 // Boost Current 升壓電路電流
#define   _GeneratorVoltage_      3 // Generator Voltage
#define   _ECBCurrent_            4 // ECB Current
#define   _GeneratorCurrent_      5 // Generator Current 預留
#define   _MainPower12V_          6 // Main Power 12V
#define   _BoostVoltage15V_       7 // Boost Voltage 15V 15V充電電壓(升壓電路)
#define   _ResistanceOffset_      8 // Resistance Offset
#define   _ECBvoltage_            9 // ECB voltage

//------------------------------------------------------------------------------
void ADC_Initial(void) ;
void ADC_ConversionProcess(void) ;
void ADC_Process(void) ;
unsigned short ADC_GetADC(unsigned char source) ;
unsigned short ADC_GetCalculatorValue(unsigned char source);
unsigned char ADC_GetDataSatndbyStatus(void) ;
unsigned short ADC_ConverterEMCurrentToADC(unsigned short iA);

#define EnableAdc()         ADC_SoftwareStartConvCmd(ADC1, ENABLE)


#endif /* __ADC_H__ */


