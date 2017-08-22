

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOOT_H
#define __BOOT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define     _DataNotMatch_        20
#define     _DataMatch_           0



#define     _FirmwareUpdateOK_    100
#define     _FlashEarseError_     101
#define     _FirmwareUpdateError_ 102


#define     _UserAPIError_        0xEF
#define     _UserAPIOk_           0xE0 
#define     _LoaderAPIError_      0xDF
#define     _LoaderAPIOk_         0xD0 


//#define     _Debuf_API_
//#define     _Debug_JHTFile_   


#define BOOT_START_ADDRESS            0x08000000  //
#define APP_START_ADDRESS             0x0801FC00  //
#define LOADER_START_ADDRESS          0x08005D00  //
#define _LInfo_BootMainStartAddress_  0x0300



/* Exported functions ------------------------------------------------------- */

char Boot_CheckUserProgram(void);
char Boot_CheckLoaderProgram(void);
void Boot_CallApplication(unsigned long ulStartAddr);
void Boot_DisableIRQn(char All);
#endif /* __BOOT_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
