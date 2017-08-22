

#ifndef __LOADERUARTCOMMAND_H__
#define __LOADERUARTCOMMAND_H__



//
#define LCB_SCF           0xD0
#define LCB_START         0xD1
#define LCB_END           0xD2
#define LCB_REV           0xD3
// Command 
// UCB --> LCB
#define		_EraseFlash_							0x10
#define		_WriteFlash_							0x12
#define		_ReadFlash_								0x14
#define		_ReadProgramState_				0x20				
#define		_FlashUnlock_							0x24
#define		_WriteCheckCode_					0x26
#define		_ReadProduceID_						0x30
#define		_ReadMcuType_							0x32
#define		_ReadMcuID_								0x34
#define		_ReadUpdateMode_					0x40
#define		_IsRequestUpdate_					0x42
#define   _ReadLoaderVersion_       0x44


// LCB --> UCB
#define		_ReturnEraseFlash_				0x11
#define		_ReturnWriteFlash_				0x13
#define		_ReturnReadFlash_					0x15
#define		_ReturnProgramState_			0x21
#define		_ReturnFlashState_				0x25
#define		_ReturnCheckCode_					0x27
#define		_ReturnProduceID_					0x31
#define		_ReturnMcuType_						0x33
#define		_ReturnMcuID_							0x35
#define		_ReturnUpdateMode_				0x41
#define		_ReturnRequestUpdate_			0x43
#define   _ReturnLoaderVersion_     0x45
#define		_ReturnInvalidCommand_		0x7F

//------------------------------------------------------------------------------


// Data
#define		_UpdateMode_							0x01
#define		_NormalMode_							0x00

#define		_LockMode_								0x00
#define		_UnlockMode_							0x01

#define		_EraseFail_								0x00
#define		_EraseOK_									0x01






#endif /* __LOADERUARTCOMMAND_H__*/


