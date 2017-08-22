/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "Incline.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned short InclineProcessTime ;
unsigned short InclineSoftStartTime ;
unsigned char  InclineSoftStartCount ; 
const unsigned short InclineSoftStartSpeed[] = {
  5,10,15,25,35,45,55,65,75,80,95,100
} ;
/* Private function prototypes -----------------------------------------------*/







/* Private functions ---------------------------------------------------------*/
void Incline_Initial(void)
{
  InclineCTR.All = 0 ;
  InclineSoftStartTime = 0 ;
  InclineSoftStartCount = 0 ;
  return ;
}




void Incline_Process(void)
{
  InclineProcessTime += 1 ;
  
  if( InclineCTR.B.StopACT == 1 )
      {
      }
  else
      {
      if( InclineCTR.B.SoftStart == 1)
          {
          InclineSoftStartTime += 1 ;
          
          }
      }
  return ;
}



