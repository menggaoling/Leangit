
#ifndef __JHTCOMM_H
#define __JHTCOMM_H

#ifdef __cplusplus
 extern "C" {
#endif
   
//
#define     _RECEIVE_OK_        0xAA
#define     _RETRY_OVER_        0xBB   
   
//
typedef unsigned char (*UartHwInputFunc)(void) ;
typedef void (*UartHwOutputFunc)(unsigned char) ;
typedef void (*UartHwDirFunc)(void) ;
typedef void (*ErrorCodeFunc)(unsigned short) ;
//

typedef struct {
  unsigned long  StartTx:1 ;
  unsigned long  Next:1 ;
  unsigned long  Tx :1 ;
  unsigned long  TxGetError:1;
  unsigned long  TxSkipError:1 ;
  unsigned long  Rx:1 ;
  unsigned long  RxCheckStartByte:1 ;
  unsigned long  ReceiveComplete:1 ;
  unsigned long  ReceiveCommandError:1 ;
  unsigned long  ReceiveLengthError:1 ;
  unsigned long  TimeoutOccur:1 ;
  unsigned long  DisconnectionOccur:1 ;
  unsigned long  ErrorOccur:1;
  unsigned long  LCBConnecttion:1 ;
  unsigned long  TxBufferError:1 ;
  unsigned long  RxBufferError:1 ;
  unsigned long  RxAnalysisBufferError:1 ;
} ControlStatus ;


typedef struct {
  unsigned char Command ;
  unsigned char Length ;
  unsigned char *pData ;
} CommPacket ;
//
typedef union {
  struct {
    unsigned char InclineDown           :1 ;
    unsigned char InclineUp             :1 ;
    unsigned char MainMotorStatus       :1 ;
    unsigned char Encoder_ACin          :1 ;
    unsigned char LowBattery            :1 ;
    unsigned char CommandError          :1 ;
    unsigned char Error                 :1 ;
    unsigned char Initial_Calibration   :1 ;
  } b ;
  unsigned char Status ;
} RxCommStatus ;
//
typedef struct {
  // Transmit 
  CommPacket        TxPacket ;
  unsigned char     *TxBuffer ; //TxBuffer[300]
  unsigned short    TxSize ;
  unsigned short    TxCount ;
  unsigned short    TxLength ;
  unsigned short    TxDelayTime ;
  unsigned short    PacketDelyTime ;
  unsigned char     RetryCount ;
  unsigned char     RetryLimit ;
  // Serial Port Data input buffer
  unsigned char     *RxBuffer ;//RxBuffer[300]
  unsigned short    RxSize ;
  unsigned char     *RxStartPoint ;
  unsigned char     *RxEndPoint ;
  //
  UartHwInputFunc   ReadData ;
  UartHwOutputFunc  SendData ;
  UartHwDirFunc     SetTXD ;
  UartHwDirFunc     SetRXD ;
  ErrorCodeFunc     SaveErrorCode ;
  // Receive Analysis 
  CommPacket        RxPacket ;
  RxCommStatus      ProtocolStatus ;
  unsigned char     *RxAnalsysBuffer ; //RxAnalsysBuffer[300]
  unsigned short    RxAnalsysSize ;
  ControlStatus     Status ;
  unsigned short    RxPacketTimeout ;
  unsigned short    RxDisConnect ;
  unsigned short    MaxRxPacketTime ;
  unsigned short    MaxRxConnectTime ;
  unsigned short    ErrorCode ;
  //
  unsigned short    DebugTxCount ;
  unsigned short    DebugRxCount ;
  unsigned short    DebugRetryCount ;
  unsigned short    DebugTimeoutCount ;
  unsigned short    DebugDisConnectCount ;
  //
} JHTCOMMLib ;


void JHTCOMM_Initial( JHTCOMMLib *CommData, unsigned char *TxAddr, unsigned short TxSize,\
    unsigned char *RxTempAddr, unsigned short RxTempSize, unsigned char *RxAddr, \
    unsigned short RxSize,unsigned short PacketDelyTime,unsigned short Timeout, \
    unsigned short Disconnect ) ;
unsigned char JHTCOMM_Communication( JHTCOMMLib *CommData ) ;
unsigned char JHTCOMM_TxData(JHTCOMMLib *CommData,unsigned char *SendData ) ;
void JHTCOMM_RxData(JHTCOMMLib *CommData,unsigned char InData) ;

//

/*
//Bike
#define CmdBikeResetECB                             0x61  // 0,0 PM���O(JIS)
#define CmdBikeSetECBPosition                       0x62  // 2,0 PM���O(JIS)           
#define CmdBikeGetRPM				                        0x63  // 0,2 ���oecb���X0~200
#define CmdBikeGetECBADC                            0x64  // 0,2 PM���O(JIS)
#define CmdBikeSetPWMPercent			                  0x65  // 2,0 �]�w�ϱ�PWM�ʤ��� 0~32768
#define CmdBikeGetBatteryStatus 	                  0x66  // 0,2 ���o�q���q���Ǧ�(H�o�q��,L�q��)0.1x255
#define CmdBikeSetWatts                             0x67  // 2,0 
#define CmdBikeSetRpmGearRatio  	                  0x68  // 2,0 �]�wrpm�t��(0.01x65535)
#define CmdBikeSetGenMegPolePare 	                  0x69  // 1,0 �o�q���Ϸ��ռ�(�w�]4)
#define CmdBikeSetLimitRpmForCharge                 0x6a  // 2,0 �]�wrpm�Ȩӱ���q���R�q 0~65535
#define CmdBikeSetLimitRpmForResis 	                0x6b  // 2,0 �]�wrpm�Ȩӱ�����O�}�� 0~65535
#define CmdBikeSetMachineType		                    0x6c  // 1,0 �]�w����
#define CmdBikeSetPowerOff			                    0x6d  // 1,0 �����q��
#define CmdBikeSetBatteryChargeLevel                0x6e  // 1,0 
#define CmdBikeSetMinResistanceWhenInclineWorking   0x6f  // 1,0

//general cmd
#define CmdInitial                                  0x70  // 0,0 �]�w�U�O��l��
#define CmdGetStatus                                0x71  // 0,0 ���A���o
#define CmdGetErrorCode                             0x72  // 0,2 ���~�N�X���o
#define CmdGetVersion                               0x73  // 0,2 ���oLCB����
#define CmdCalibrate                                0x74  // 0,0 �۰ʮե�
#define CmdUpdateProgram                            0x75  // 2,1
#define CmdSkipCurrentError                         0x76  // 0,0 �����ثe���~�X
#define CmdSetSpecialExtend                         0x77  // 1,1
#define CmdGetDCIVersion                            0x78  // 0,27
#define CmdGetDCIEnvironment                        0x79  // 0,84

//incline
#define CmdSetInclinePercent                        0x80  // 2,0
#define CmdBikeGetInclineLocation                   0x81  // 0,2
#define CmdBikeSetEcbAction			                    0x82  // 1,0
#define CmdBikeGetECBStatus                         0x83  // 0,1 PM(JIS)
#define CmdBikeGetECBCount                          0x84  // 0,2 PM(JIS)
#define CmdBikeSetEMagnetCurrent	                  0x85  // 2,0
#define CmdBikeSetResistanceType                    0x86  // 2,0
#define CmdSetBeginBatteryCharge                    0x87  // 2,0
#define CmdGetBatteryCapacity                       0x88  // 0,2

#define CmdSetMotorSpecification                    0x90  // 2,0
#define CmdGetDriverType                            0x91  // 0,1 ���o���F�X�ʾ�����
#define CmdGetDCIMaxDataLength                      0x92  // 0,1 DCI�M��command
#define CmdSetMotorPowerOff                         0x93  // 1,0
#define CmdSetDCIForceInclineOperation              0x94  // 1,0
#define CmdGetTreadmillInUsed                       0x95  // 0,1
#define CmdGetMainMotorInformation                  0x96  // 0,10


#define CmdGetDCIInclineRange	                      0xA0	// 0,4
#define CmdSetDCIInclineRange                       0xA1	// 4,0
#define CmdGetDCIRpmSpeedRange	                    0xA6	// 0,4
#define CmdSetDCIRpmSpeedRange                      0xA7	// 8,0	
#define CmdSetDCIManualCalbration                   0xAB	// 0,0	


//==> Digital for TM
#define CmdSetAndGetMotorRPM                        0xf0  // 2,2
#define CmdSetMotorRPM                              0xf1  // 2,0 �]�w���F��t
#define CmdSetPwmAddStep                            0xf2  // 2,0 �]�w�[�t�t�v
#define CmdSetPwmDecStep                            0xf3  // 2,0 �]�w��t�t�v
#define CmdSetPemStopStep                           0xf4  // 2,0 �]�w����t�v
#define CmdSetInclineAction                         0xf5  // 1,0 �]�w�ɭ�����
#define CmdSetInclineLocation                       0xf6  // 2,0 �]�w�ɭ���m
#define CmdSetWorkStatus                            0xf7  // 1,0 �]�w�B�ʪ��A
#define CmdGetRollerRpm                             0xf8  // 0,2 ���o�u���t��
#define CmdGetMotorRpm                              0xf9  // 0,2 ���o���F�t��
#define CmdGetInclineLocation                       0xfa  // 0,2 ���o�ɭ����
#define CmdTuneEndPointIncline                      0xfb  // 2,0
#define CmdTuneEndPointIncline2                     0xfc  // 2,0
#define CmdSetInclineStroke                         0xfd  // 2,0
#define CmdSetCompensationVoltage                   0xfe  // 2,0 �]�w���v�q�� 0 ~ 65535


*/


#ifdef __cplusplus
}
#endif

#endif /* __JHTCOMM_H */



