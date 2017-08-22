

#ifndef __LOADER_H__
#define __LOADER_H__


//------------------------------------------------------------------------------
#define   LCB_StartAddr               ((uint32_t)0x08006000)
#define   LCB_EndAddr                 ((uint32_t)0x0801FFFF)
#define   LCB_INFO_Addr               ((uint32_t)0x0801FC00)
#define   NVIC_VectTab_LOADER         0x08005D00 

// Infomation 
#define   _LInfo_LoaderStartAddress_          0x5F00
#define   _LInfo_ManufactureIndex_            _LInfo_LoaderStartAddress_ + 4
#define   _LInfo_McuTypeIndex_                _LInfo_ManufactureIndex_ + 40
#define   _LInfo_ModuleNameIndex_             _LInfo_McuTypeIndex_ + 20
#define   _LInfo_ModuleNoIndex_               _LInfo_ModuleNameIndex_ + 20
#define   _LInfo_ProductIndex_                _LInfo_ModuleNoIndex_ + 20
#define   _LInfo_VersionIndex_                _LInfo_ProductIndex_ + 20


//
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)

/* Flash Access Control Register bits */
#define ACR_LATENCY_Mask         ((uint32_t)0x00000038)
#define ACR_HLFCYA_Mask          ((uint32_t)0xFFFFFFF7)
#define ACR_PRFTBE_Mask          ((uint32_t)0xFFFFFFEF)

/* Flash Access Control Register bits */
#define ACR_PRFTBS_Mask          ((uint32_t)0x00000020) 

/* Flash Control Register bits */
#define CR_PG_Set                ((uint32_t)0x00000001)
#define CR_PG_Reset              ((uint32_t)0x00001FFE) 
#define CR_PER_Set               ((uint32_t)0x00000002)
#define CR_PER_Reset             ((uint32_t)0x00001FFD)
#define CR_MER_Set               ((uint32_t)0x00000004)
#define CR_MER_Reset             ((uint32_t)0x00001FFB)
#define CR_OPTPG_Set             ((uint32_t)0x00000010)
#define CR_OPTPG_Reset           ((uint32_t)0x00001FEF)
#define CR_OPTER_Set             ((uint32_t)0x00000020)
#define CR_OPTER_Reset           ((uint32_t)0x00001FDF)
#define CR_STRT_Set              ((uint32_t)0x00000040)
#define CR_LOCK_Set              ((uint32_t)0x00000080)

/* FLASH Mask */
#define RDPRT_Mask               ((uint32_t)0x00000002)
#define WRP0_Mask                ((uint32_t)0x000000FF)
#define WRP1_Mask                ((uint32_t)0x0000FF00)
#define WRP2_Mask                ((uint32_t)0x00FF0000)
#define WRP3_Mask                ((uint32_t)0xFF000000)

/* FLASH Keys */
#define RDP_Key                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

/* Delay definition */   
#define EraseTimeout             ((uint32_t)0x000B0000)
#define ProgramTimeout           ((uint32_t)0x00002000)


void Loader_Unlock(void);
FLASH_Status Loader_ErasePage(uint32_t Page_Address) ;
FLASH_Status Loader_ProgramHalfWord(uint32_t Address, uint16_t Data) ;
void Loader_ClearFlag(uint16_t FLASH_FLAG);
FLASH_Status Loader_GetStatus(void);
FLASH_Status Loader_WaitForLastOperation(uint32_t Timeout) ;
FLASH_Status Loader_EarseUserProgram(uint32_t SAddr,uint32_t EAddr) ;
FLASH_Status Loader_EarseLCBProgram(void) ;


#endif /* __LOADER_H__*/


