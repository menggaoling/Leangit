

#ifndef __BOOTUARTCOMMAND_H__
#define __BOOTUARTCOMMAND_H__



//
#define LCB_SCF           0xD0
#define LCB_START         0xD1
#define LCB_END           0xD2
// Command 
// UCB --> LCB
#define		_EraseFlash_		0x10
#define		_WriteFlash_		0x12
#define		_ReadFlash_		0x14
#define		_ReadProgramState_	0x20				
#define		_FlashUnlock_		0x24
#define		_WriteCheckCode_	0x26
#define		_ReadProduceID_		0x30
#define		_ReadMcuType_		0x32
#define		_ReadMcuID_		0x34
#define		_ReadUpdateMode_	0x40
#define		_IsRequestUpdate_	0x42


// LCB --> UCB
#define		_ReturnEraseFlash_	0x11
#define		_ReturnWriteFlash_	0x13
#define		_ReturnReadFlash_	0x15
#define		_ReturnProgramState_	0x21
#define		_ReturnFlashState_	0x25
#define		_ReturnCheckCode_	0x27
#define		_ReturnProduceID_	0x31
#define		_ReturnMcuType_		0x33
#define		_ReturnMcuID_		0x35
#define		_ReturnUpdateMode_	0x41
#define		_ReturnRequestUpdate_	0x43
#define		_ReturnInvalidCommand_	0x7F

//------------------------------------------------------------------------------
// START CMD Length Data CheckSum END
// MCU_A --> MCU_B
#define   _FeedbackADC_                     0xA0    // DATA: [GE_V][GE_A][E_A][12V_V1][ERROR]
#define   _FeedbackDefaultVolue_            0xA1    // DATA: [EM_MAX][EM_R]
#define   _FeedbackUpdateStatus_            0xA2    // DATA: STATUS
                                                    //       'Y' = Yes
                                                    //       'N' = No ;
#define   _FeedbackUpdateLoaderStatus_      0xA3    // DATA: STATUS
                                                    //       'Y' = Yes
                                                    //       'N' = No ;
#define   _FeedbackReadFlashMemoryData_     0xA4    //
#define   _FeedbackReadVersionData_         0xA5    //


// MCU_A <-- MCU_B
#define   _SetEMControlCurrent_             0xB0    // DATA: [EM_A]
#define   _SetDefaultVolue_                 0xB1    // DATA: [EM_MAX][EM_R]
#define   _JustUpdateAPI_                   0xB2
#define   _JustUpdateLoaderAPI_             0xB3
#define   _ReadFlashMemoryData_             0xB4
#define   _ReadVersion_                     0xB5

#define   _TestSetEMPWM_                    0xBF    // DATA: [TEST_PWM]
//------------------------------------------------------------------------------


// Data
#define		_UpdateMode_		0x01
#define		_NormalMode_		0x00

#define		_LockMode_		0x00
#define		_UnlockMode_		0x01

#define		_EraseFail_		0x00
#define		_EraseOK_		0x01






#endif /* __BOOTUARTCOMMAND_H__*/


