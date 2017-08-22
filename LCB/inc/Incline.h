#ifndef __INCLINE_H__
#define __INCLINE_H__

typedef union{
  struct {
    unsigned short stopST:1 ;
    unsigned short upST:1 ;
    unsigned short downST:1 ;
    unsigned short rev:5      
    unsigned short SoftStart:1 ;
    unsigned short StopACT:1 ;
    unsigned short UpACT:1 ;
    unsigned short DownACT:1 ;
    unsigned short MoveACT:1 ;
    unsigned short StopDelay:1 ;
    unsigned short AutoCalibration:1 ;
    unsigned short MotorDirection:1 ;
  } B ;
  unsigned short All ;
} InclineDataType ;


__no_init InclineDataType InclineCTR ;


/* Private function prototypes -----------------------------------------------*/
void Incline_Initial(void) ;


#endif /* __INCLINE_H__ */


