#ifndef __RPM_H__
#define __RPM_H__

typedef struct {
  unsigned short *GearRate ;
  unsigned char  *GenMegPolePair ;
  unsigned short AverageRPM ;
  float Frequency ;
} RPMDataStruct ;

/* Private function prototypes -----------------------------------------------*/
void RPM_Initial(void) ;
void RPM_CaptureInput(void) ;
void RPM_CaptureTimeoutProcess(void);
unsigned short RPM_CalculatorAnalogRPM(void) ;



//
extern RPMDataStruct RPMData ; 
#endif /* __RPM_H__ */


