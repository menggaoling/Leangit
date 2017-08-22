#ifndef __JHTCOMMAND_H__
#define __JHTCOMMAND_H__


//############################################################################
// Johnson RS-485/422 Communication Protocol 
//	Version: 20070511
//	Release Date: 20070720
//----------------------------------------------------------------------------
// Description:
// <UCB->LCB>
// Start(1Byte)+Address(1Byte)+Command(1Byte)+Length(1Byte)+Data(nByte)+CRC(1Byte)
// #Start:>0x00
// #Address:>0xFF
//
// <LCB->UCB>
// Start(1Byte)+Status(1Byte/4Byte)+Command(1Byte)+Length(1Byte)+Data(nByte)+CRC(1Byte)
//	#Start:>0x01
//	#Data:>nByte, 1Word:>(High Byte,Low Byte)
//
//	Timeout : 150ms
//	Rx/Tx Change Delay: 5ms
//
//
//
//
//
//----------------------------------------------------------------------------
//	Include File Version:
// Release Date: 20070831	
//############################################################################





/* ====================================================================== */
// Register Define
// for uart package
/*
  uart package
    1st          2nd             3rd        4th      5th           6th
   ____________________________________________________________________
  | start | Address / Status | Commmand | Length | Data(0~ 255  | CRC8 |
  |_______|__________________|__________|________|_byte )_______|______|
*/

// UCB
#define	UCBStartByte  							0
#define	StartByte           			0  	// first byte in package
#define	AddrStatusByte      			1  	// 2nd byte in package
#define	CommandByte         			2  	// 3rd byte in package
#define	LengthByte          			3  	// 4th byte in package
#define	DataMsbLocation      			4  	// 5th byte in package
#define	LengthPacketExcludeData 		5		//EndPackageLocation   5  

// LCB
#define	LCBStartByte	1


// Command Define
// <Common commands>--------------------------------<UCB>-<LCB>- 
#define	CmdInitial      	          0x70	// 	0		0			
#define	CmdGetStatus    	          0x71	//	0		0			
#define	CmdGetErrorCode 	          0x72	//	0		2			
#define	CmdGetVersion   	          0x73	//	0		2			
#define	CmdCalibrate 		          0x74  //	0		0			
#define	CmdUpdateProgram	          0x75	//	2		1				
#define	CmdSkipErrorCode 	          0x76	//	0		0			
#define	CmdSpecialExtCommand	          0x77	//	1		1	
#define	CmdGetProtocolVersion	          0x7C	//	0		2	
// < Commands of TM >
#define	CmdSetMotorRpmAndReturn		  0xF0	//	2		2						
#define	CmdSetMotorRpm			  0xF1	//	2		0	
#define	CmdSetPwmAddStep		  0xF2	//	2		0	
#define	CmdSetPwmDecStep		  0xF3	//	2		0	
#define	CmdSetPwmStopStep		  0xF4	//	1		0	
#define	CmdSetInclineAction   		  0xF5  // 	1		0	
#define	CmdSetInclineLocation 		  0xF6  // 	2		0									
#define	CmdSetWorkStatus      		  0xF7	//	1		0						
#define	CmdGetRollerRpm       		  0xF8	//	0		2	
#define	CmdGetMotorRpm        		  0xF9	//	0		2
#define	CmdGetInclineLocation 		  0xFA  // 	0		2
#define	CmdSetGapVrCalibrateIncline 	  0xFB  // 	2		0
#define	CmdSetGapVrCalibrateIncline2 	  0xFC	//	2		0
#define	CmdSetInclineStroke  		  0xFD	//	2		0
#define	CmdSetResisVolt       		  0xFE	//	2		0
#define	CmdSetDriveMotorHP		  0x90	//	2		0
// < Commands of bike >
#define	CmdSetEcbInit     		  0x61  // 	0		0
#define	CmdSetEcbLocation 		  0x62  // 	2		0
#define	CmdGetRpm         		  0x63	//	0		2
#define	CmdGetEcbLocation 		  0x64  // 	0		2
#define	CmdSetPwm         		  0x65	//	2		0
#define	CmdGetBatteryStatus		  0x66	//	0		2
#define	CmdSetWatts			  0x67	//	2		0
#define	CmdSetRpmGearRatio    		  0x68	//	2		0
#define	CmdSetGenMegPolePair  		  0x69	//	1		0
#define	CmdSetLimitRpmForCharge 	  0x6A	//	2		0	
#define	CmdSetLimitRpmForResis 		  0x6B	//	2		0
#define	CmdSetMachineType  		  0x6C	//	1		0
#define	CmdSetPowerOff 			  0x6D	//	1		0
#define	CmdSetBatteryCharge  		  0x6E	//	1		0
#define	CmdSetZeroResisWhenInclineWorking 0x6F	//	1		0
#define	CmdEUPsMode			  0x7A	//	1		1				UCB-> 0: EUPs OFF,FF: EUPs ON  LCB-> 1:Command action	
#define	CmdSetInclinePercent  		  0x80	//	2		0
#define	CmdGetInclinePercent  		  0x81 	//	0		2
#define	CmdSetEcbAction			  0x82	// 	1		0
#define	CmdGetEcbStatus			  0x83	// 	0		1
#define	CmdGetEcbCount			  0x84	// 	0		2
#define	CmdSetEMagnetCurrent		  0x85	//	2		0
#define	CmdSetResistanceTypeAndResistance 0x86	//	2		0
#define	CmdSetBeginBatteryCharge	  0x87  //  2   0     
#define CmdGetBatteryCapacity             0x88  //  0   2      0~100%
#define	CmdSetClimbmillStatus		  0x8A	//  1   0
#define	LCB_Workout_Start		  0x00	// 
#define	LCB_Workout_Stop		  0x01	//
#define	LCB_EStop_Disable		  0xA0	//
#define	LCB_EStop_Enable		  0xA1	//
#define	LCB_EStop_Unlock		  0xA5	//
#define	CmdGetClimbmillStatus		  0x8B  //  0   1
#define	CmdSetEStopActionValue		  0x8C	//	1		0
// LCB Data 
#define	CmdLCBDeviceData		  0x50	//  3
#define	GetEEPromMemorySizes		  0x01	// 	3		2
#define	GetEEPromMemoryData		  0x02	//	3		2
#define	GetECBCurrent			  0x03	//  3   4
#define	GetEStopCapacitance		  0x04  //  3   4
#define GetDCBusStatus                    0x05  //  3   4
#define GetLCBVersion                     0x06  //  3   8
#define GetLCBInformation                 0xF0  //  5
// Updata Loader  
#define LCBSoftwareReset                  0x70  //  1
#define EraseLoaderMemory                 0x71  //  1
#define WriteWordLoaderData               0x72  //  6+50(Data) 1
#define ReadByteFlashData                 0x73  //
// Machine Type Define Data
#define	_BikeEP_		0
#define	_Setpper_		1
#define	_Ramp_IncluneEP_	2
#define	_SwingInclineEP_	3
#define	_Treadmill_		4
#define	_Climbmill_		5
// Incline Manual Action
#define ManualInclineUp                   0x01
#define ManualInclineDown                 0xFF
#define ManualInclineStop                 0x00
// Resistance Type Define Data
#define	_ElectroMagnet_Device_	0
#define	_Resistor_Device_	1
// LCB Type and Version
#define LCB_Type 0x14
#define LCB_VersionNumber   1
#define LCB_FormalVersion   2
#define LCB_BetaVersion     2
//
// Protocol Version
#define LCB_ProtocolVersion 530 //V5.3
#define NewVersion          1
#define OldVersion          0
#endif /* __JHTCOMMAND_H__ */
