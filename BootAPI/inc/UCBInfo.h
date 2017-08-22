

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UCBINFO_H
#define __UCBINFO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
// Infomation 
//#pragma location = ".bootinfo"
__root const unsigned char Boot_Manufacture[40] = "Johnson Health Tech. Co., Ltd." ;
__root const unsigned char Boot_McuType[20] = "STM32F102RB" ;
__root const unsigned char Boot_ModuleName[20] = "1xBikeLCB";//"LCB-A" ;
__root const unsigned char Boot_ModuleNo[20] = "L13CA1";//"L118A1_MCU-B" ;
__root const unsigned char Boot_Product[20] = "Matrix" ;
__root const unsigned char Boot_Version[20] = "V001-20140425";//"S001-20120723" ;

// array size 必須是四的倍數
__root const unsigned short User_LCBVersion       @ ".userinfo" = 0x1400;//= 0x1400; //LCB Type 0x14 
__root const unsigned short User_revSpace         @ ".userinfo" = 0xAAAA ;
__root const unsigned long  User_CCodeStartAddr   @ ".userinfo" = 0x6000 ;
__root const unsigned long  User_CCodeEndAddr     @ ".userinfo" = 0x1FFFF ;
__root const unsigned short User_CCodeBlockSize   @ ".userinfo" = 7168 ;
__root const unsigned short User_CCodeBlockCount  @ ".userinfo" = 15 ;
__root const unsigned long  User_FlashOffset      @ ".userinfo" = 0x08000000 ; 
__root const unsigned long  User_APIStartAddr     @ ".userinfo" = 0x08006000 ; 
__root const unsigned long  User_APIEndAddr       @ ".userinfo" = 0x0801FBFF ; 
__root const unsigned long  User_InfoStartAddr    @ ".userinfo" = 0x0801FC00 ; 
__root const unsigned long  User_InfoEndAddr      @ ".userinfo" = 0x0801FFFF ;
__root const unsigned char  User_Manufacture[40]  @ ".userinfo" = "Johnson Health Tech. Co., Ltd." ;
__root const unsigned char  User_McuType[20]      @ ".userinfo" = "STM32F102RB" ;
__root const unsigned char  User_ModuleName[20]   @ ".userinfo" = "1xBikeLCB";//"LCB-A" ;
__root const unsigned char  User_ModuleNo[20]     @ ".userinfo" = "L13CA1";//"L118A1_MCU-B" ;
__root const unsigned char  User_Product[20]      @ ".userinfo" = "Matrix" ;
__root const unsigned char  User_Version[20]      @ ".userinfo" = "V001-20140425";//"S001-20140307" ;
__root const unsigned char  User_CheckCode[16]    @ ".userinfo" ={ 0xFF	,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                                                                   0xFF	,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF	
                                                                 } ;
__root const unsigned short User_ProgramStatus    @ ".userinfo" = 0xFFFF ;
__root const unsigned short User_revSpace1        @ ".userinfo" = 0x5555 ;

// Infomation 
#define   _UInfo_StartAddress_            0x1FF00
#define   _UInfo_LCBVersionIndex_             _UInfo_StartAddress_
#define   _UInfo_revSpace_                    _UInfo_LCBVersionIndex_ + 2
#define   _UInfo_CCodeStartAddr_              _UInfo_revSpace_ + 2
#define   _UInfo_CCodeEndAddr_                _UInfo_CCodeStartAddr_ + 4
#define   _UInfo_CCodeBlockSize_              _UInfo_CCodeEndAddr_ + 4
#define   _UInfo_CCodeBlockCount_             _UInfo_CCodeBlockSize_ + 2
#define   _UInfo_FlashOffset_                 _UInfo_CCodeBlockCount_ + 2
#define   _UInfo_APIStartAddr_                _UInfo_FlashOffset_ + 4
#define   _UInfo_APIEndAddr_                  _UInfo_APIStartAddr_ + 4
#define   _UInfo_InfoStartAddr_               _UInfo_APIEndAddr_ + 4
#define   _UInfo_InfoEndAddr_                 _UInfo_InfoStartAddr_ + 4
//
#define   _UInfo_ManufactureIndex_            _UInfo_InfoEndAddr_ + 4
#define   _UInfo_McuTypeIndex_                _UInfo_ManufactureIndex_ + 40
#define   _UInfo_ModuleNameIndex_             _UInfo_McuTypeIndex_ + 20
#define   _UInfo_ModuleNoIndex_               _UInfo_ModuleNameIndex_ + 20
#define   _UInfo_ProductIndex_                _UInfo_ModuleNoIndex_ + 20
#define   _UInfo_VersionIndex_                _UInfo_ProductIndex_ + 20
#define   _UInfo_CheckCodeIndex_              _UInfo_VersionIndex_ + 20
#define   _UInfo_ProgramStatusIndex_          _UInfo_CheckCodeIndex_ + 16
/*
#define   _UInfo_LCBVersionIndex_             _UInfo_ProgramStatusIndex_+2
#define   _UInfo_CCodeStartAddrIndex_         _UInfo_LCBVersionIndex_+2
#define   _UInfo_CCodeEndAddrIndex_           _UInfo_CCodeStartAddrIndex_+4
#define   _UInfo_CCodeBlockSizeIndex_         _UInfo_CCodeEndAddrIndex_+4
#define   _UInfo_CCodeBlockCountIndex_        _UInfo_CCodeBlockSizeIndex_+2
*/

// Infomation 
#define   _LInfo_LoaderStartAddress_          0x5F00
#define   _LInfo_ManufactureIndex_            _LInfo_LoaderStartAddress_ + 4
#define   _LInfo_McuTypeIndex_                _LInfo_ManufactureIndex_ + 40
#define   _LInfo_ModuleNameIndex_             _LInfo_McuTypeIndex_ + 20
#define   _LInfo_ModuleNoIndex_               _LInfo_ModuleNameIndex_ + 20
#define   _LInfo_ProductIndex_                _LInfo_ModuleNoIndex_ + 20
#define   _LInfo_VersionIndex_                _LInfo_ProductIndex_ + 20
#define   _LInfo_PasswordIndex_               _LInfo_VersionIndex_ + 20
#define   _LInfo_LCBVersionIndex_             _LInfo_PasswordIndex_ + 20




#endif /* __UCBINFO_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
