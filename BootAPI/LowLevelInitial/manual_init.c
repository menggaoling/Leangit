/******************************************************************************
.................... MATRIX A1/A3 software history ............................ 
.. S001-beta-01 ..
.. 99.03.29 ..



*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include  <stdio.h>
#include  <stdarg.h>
#include  "stm32f10x_conf.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Declare the sections. */
/*
#pragma section = ".special"
#pragma section = ".special_init"
void UCB_memoryInitial(void) 
{
  uint32_t size;
  uint32_t *targetAddr = __section_begin(".special");
  uint32_t const *sourceAddr = __section_begin(".special_init");
  uint32_t const *sourceAddrEnd = __section_end(".special_init");
  size = sourceAddrEnd - sourceAddr;
  do
  {
    *targetAddr++ = *sourceAddr++;
    size -= 1;
  } while (size != 0);
  return ;
}


#pragma section = ".bss"
void UCB_memoryInitialZero(void) 
{
  uint32_t size;
  uint32_t *targetAddr = __section_begin(".bss");
  uint32_t *targetAddrEnd = __section_end(".bss");
  size = targetAddrEnd - targetAddr;
  do
  {
    *targetAddr++ = 0 ;
    size -= 1;
  } while (size != 0);

  return ;
}
*/
typedef uint32_t const * init_fun_t(uint32_t const *);

void __manual_data_init(void)
{
  uint32_t const * p = (uint32_t const *)0x0801F800 ;

  while( *p != 0 && *p != 0xFFFFFFFF )
      {
      init_fun_t * fun = (init_fun_t *)*p++;
      p = fun(p) ;
      }
 
  return ;
}  
//------------------------------------------------------------------------------

