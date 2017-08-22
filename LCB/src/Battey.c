//##############################################################################


//##############################################################################
/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "ADC.h"
#include  "LCBMain.h"
#include  "MultiMCUComm.h"



extern LCBADValueStruct  LCBADC ;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned short BatteryProcessTimeCounter ;
union {
  struct {
    unsigned short Enable:1  ;                           // bit 0
    unsigned short Charger:1 ;                           // bit 1
    unsigned short Discharger:1 ;             
    unsigned short 
  } bit ;
  unsigned short All;
} Battery_CSR ;

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Baterry_Initial(void)
{
  BatteryProcessTimeCounter = 0 ;
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Battery_Process(void)
{
  return ;
}







