/******************** (C) COPYRIGHT 2009 Johnson Fitness Inc. ******************
* File Name          : EE93cxx.c
* Author             : 
* Version            : V1.0.0
* Date               : 01/13/2009
* Description        : 
*******************************************************************************/

//##############################################################################
// EEPROM AT93Cxx Serial Program
// -----------------------------
// AT93C86 Command Define
// ----------------------------------------------------------------------------
// Command 		Format
//					  SB  OP		Address 				          Data
//					      CODE	x8			      x16		      x8			x16		    Comments
//-----------------------------------------------------------------------------
//	READ 			1 	10 	  A10-A0 	      A9-A0								          Reads data stored 
//																				                          in memory, at
//																				                          specified address.
//-----------------------------------------------------------------------------
//	EWEN 			1 	00 	  11XXXXXXXXX 	11XXXXXXXX							      Write enable must 
//																				                          precede all
//																				                          programming modes.
//-----------------------------------------------------------------------------
//	ERASE 		1 	11 	  A10-A0 	      A9-A0 							          Erase memory 
//																				                          location An - A0.
//-----------------------------------------------------------------------------
//  WRITE			1	  01	  A10-A0		    A9-A0		    D7-D0		D15-D0	  Writes memory 
//																				                          location An - A0. 
//-----------------------------------------------------------------------------
//	ERAL 			1 	00 	  10XXXXXXXXX 	10XXXXXXXX							      Erases all memory 
//																				                          locations. Valid
//																				                          only at VCC=4.5-5.5V
//-----------------------------------------------------------------------------
//	WRAL 			1 	00 	  01XXXXXXXXX 	01XXXXXXXX 	D7-D0 	D15-D0	  Writes all memory 
//																				                          locations. Valid
//																				                          only at VCC=4.5-5.5V
//-----------------------------------------------------------------------------
//	EWDS 			1 	00 	  00XXXXXXXXX 	00XXXXXXXX 							      Disables all 
//																				                          programming 
//																				                          instructions.
//-----------------------------------------------------------------------------
//
//<SYMBOL>                < 93Cxx Pin Assign>       	< MCU Pin Assign>
//CS_EErom-->93Cxx Pin1:CS,Chip Select	              PB_15	// O
//SK_EErom-->93Cxx Pin2:CLK,Clock		      PB_14	// O
//DO_EErom-->93Cxx Pin3:DI,Data Input		      PB_13	// O
//DI_EErom-->93Cxx Pin4:DO,Data Output	              PB_12	// I 
//##############################################################################
/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "PinDefine.h"
#include  "EE93CXX.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void EE93CXX_EARSE(unsigned short) ;
void EE93CXX_WriteByte(unsigned short,unsigned char) ;
unsigned char EE93CXX_ReadByte( unsigned short ) ;

void EE93CXX_EWEN(void) ;       
void EE93CXX_EWDS(void) ;  
void EE93CXX_EARSE(unsigned short) ;
void EE93CXX_WriteByte(unsigned short,unsigned char) ;
unsigned char EE93CXX_ReadByte( unsigned short ) ;
void EE93CXX_WriteAddr( unsigned short ) ;
void EE93CXX_WriteData( unsigned char ) ;
unsigned char EE93CXX_ReceiveByte(void) ;
void EE93CXX_SK(void) ;


//*****************************************************************************
//
//*****************************************************************************
void EE93CXX_WriteBlockToEeprom( unsigned short addr , unsigned short count, unsigned char *DataBuffer  ) 
{
  unsigned short DataCount ;
//
  CS_EErom(HIGH) ;
  EE93CXX_EWEN() ;
  for( DataCount = 0 ; DataCount < count ; DataCount++ )
  {
      EE93CXX_EARSE( (addr+DataCount) ) ;  
      EE93CXX_WriteByte( (addr+DataCount) , DataBuffer[DataCount] ) ;
  } 
  EE93CXX_EWDS() ;            
  CS_EErom(LOW) ;
  SK_EErom(LOW) ;
  return ;              
}


//*****************************************************************************
// Read EEPROM ALL DATA to Memory
//*****************************************************************************
void EE93CXX_ReadBlockFromEeprom(unsigned short addr , unsigned short count, unsigned char *DataBuffer )
{
	unsigned short DataCount ;
	 
	for( DataCount = addr ; DataCount < ( addr+count ) ; DataCount++ )
	   {
	   DataBuffer[DataCount-addr] = EE93CXX_ReadByte( DataCount ) ;
	   } 
	return ;                  
}




//*****************************************************************************
// EEPROM Write Data
//
//*****************************************************************************
void EE93CXX_WriteDataToEeprom(unsigned short addr , unsigned char count , unsigned char *DataBuffer) 
{
  unsigned short DataCount ;
  //
	CS_EErom(HIGH) ;
	EE93CXX_EWEN() ;
  //
  for( DataCount = addr ; DataCount < (addr+count) ; DataCount++ )
	   {
     EE93CXX_EARSE( DataCount ) ;  
	   EE93CXX_WriteByte( DataCount , DataBuffer[(DataCount-addr)] ) ;
	   } 
  //
	EE93CXX_EWDS() ;            
	CS_EErom(LOW) ;
	SK_EErom(LOW) ;
  //
	return ;
}       


//*****************************************************************************
// EEPROM Read Data
//
//*****************************************************************************
void EE93CXX_ReadDataFromEeprom(unsigned short adr ,unsigned char count , unsigned char *ReturnDataBuffer ) 
{
 	unsigned short DataCount ;

  CS_EErom(HIGH) ;
  DO_EErom(HIGH) ;
  EE93CXX_SK( ) ;
  DO_EErom(HIGH) ;
  EE93CXX_SK( ) ;
  DO_EErom(LOW) ;
  EE93CXX_SK( ) ;
  
  EE93CXX_WriteAddr(adr) ;
  DO_EErom(LOW) ;
  
  for( DataCount = 0 ; DataCount < count ; DataCount++ )
      ReturnDataBuffer[DataCount] = EE93CXX_ReceiveByte() ;
  
  CS_EErom(LOW) ;
  SK_EErom(LOW) ;	
}

//*****************************************************************************
// Write Enable
// ============
//
// Command 		Format
//					SB OP		Address 				Data
//						CODE	x8			x16		x8			x16		Comments
//-----------------------------------------------------------------------------
//	EWEN 			1 	00 	11XXXXX 	11XXXX							Write enable must 
//																				precede all
//																				programming modes.
//-----------------------------------------------------------------------------
void EE93CXX_EWEN( )
{
	CS_EErom(HIGH) ;
	DO_EErom(HIGH); 
	EE93CXX_SK( ) ;
	DO_EErom(LOW) ; 
	EE93CXX_SK( ) ;
	DO_EErom(LOW) ;
	EE93CXX_SK( ) ;                 
	EE93CXX_WriteAddr( 0x600 ) ;    
	CS_EErom(LOW) ;
	DO_EErom(LOW) ;   
	CS_EErom(HIGH) ;
	CS_EErom(LOW) ;
	return ;
}       



//*****************************************************************************
//	Erase EEPROM 
// ============
//
// Command 		Format
//					SB OP		Address 				Data
//						CODE	x8			x16		x8			x16		Comments
//-----------------------------------------------------------------------------
//	ERASE 		1 	11 	A6-A0 	A5-A0 							Erase memory 
//																				location An - A0.
//
// <Time chart>
//
//				+------------------------------------------
//				|	
// 		---+
//-----------------------------------------------------------------------------
void EE93CXX_EARSE(unsigned short adr)
{
  unsigned long DelayTimeCount = 0 ;
	CS_EErom(HIGH) ;
	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;
	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;
	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;

	EE93CXX_WriteAddr( adr ) ;
	CS_EErom(LOW) ;
	SK_EErom(LOW) ;
	DO_EErom(LOW) ;
	//--------------------------------------------------------------------------
	// Check Status
	CS_EErom(HIGH) ;
	while( DI_EErom() == 0 && DelayTimeCount < 2000 )
      {
      DelayTimeCount += 1 ;
      }
  //
	CS_EErom(LOW) ;
	//--------------------------------------------------------------------------    
	return ;
}


//*****************************************************************************
// Write Data To EEPROM
// ====================
//
// Command 		Format
//					SB OP		Address 				Data
//						CODE	x8			x16		x8			x16		Comments
//-----------------------------------------------------------------------------
// WRITE			1	01		A6-A0		A5-A0		D7-D0		D15-D0	Writes memory 
//																				location An - A0. 
//-----------------------------------------------------------------------------
void EE93CXX_WriteByte(unsigned short adr,unsigned char value)
{  
  unsigned long DelayTimeCount = 0 ;
	CS_EErom(HIGH) ;
	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;
	DO_EErom(LOW) ;
	EE93CXX_SK( ) ;
	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;
	EE93CXX_WriteAddr( adr ) ;
	EE93CXX_WriteData( value ) ;    
	CS_EErom(LOW) ;
	DO_EErom(LOW) ;
	SK_EErom(LOW) ;
	//--------------------------------------------------------------------------
	// Check Status
	CS_EErom(HIGH) ;
	while( DI_EErom() == 0 && DelayTimeCount < 2000 )
      {
      DelayTimeCount += 1 ;
      }
	CS_EErom(LOW) ;
	//--------------------------------------------------------------------------  
	return ;     
}               



//*****************************************************************************
// Disable All Program Instructions
// ================================
// Command 		Format
//					SB OP		Address 				Data
//						CODE	x8			x16		x8			x16		Comments
//-----------------------------------------------------------------------------
//	EWDS 			1 	00 	00XXXXX 	00XXXX 							Disables all 
//																				programming 
//																				instructions.
//-----------------------------------------------------------------------------
void EE93CXX_EWDS( )
{
	CS_EErom(HIGH) ;

	DO_EErom(HIGH) ;
	EE93CXX_SK( ) ;

	DO_EErom(LOW) ;
	EE93CXX_SK( ) ;

	DO_EErom(LOW) ; 
	EE93CXX_SK( ) ;

	EE93CXX_WriteAddr( 0 ) ;
	CS_EErom(LOW) ;
	DO_EErom(LOW) ;
	SK_EErom(LOW) ;
	CS_EErom(HIGH) ;
	CS_EErom(LOW) ;
	return ;
}


//*****************************************************************************
// Write Address Sub-Program
// =========================
//
// AT93C46 		<A5-A0>
// AT93C56/66	<A7-A0>
// AT93C86    <A10-A0>
//*****************************************************************************
void EE93CXX_WriteAddr( unsigned short value )
{ 
	unsigned char DataCount ;
  unsigned short OutputData ;

  OutputData = value << 5 ;
	for( DataCount = 0 ; DataCount < 11 ; DataCount++ )
      {
      //
      DO_EErom(LOW);  
      if( (OutputData & 0x8000) != 0 )
          DO_EErom(HIGH);
      //
      OutputData = OutputData << 1 ;
      EE93CXX_SK( ) ;
      //
      }
  
	return ;
}       


//*****************************************************************************
// Write Data Sub-Program
//*****************************************************************************
void EE93CXX_WriteData( unsigned char value )
{ 
	unsigned char DataCount ;
  unsigned char OutputData ;
  
  OutputData = value ;
  for( DataCount = 0 ; DataCount < 8 ; DataCount++ )
      {
      //
      DO_EErom(LOW);  
      if( (OutputData & 0x80) != 0 )
          DO_EErom(HIGH);
      //
      OutputData = OutputData << 1 ;
      EE93CXX_SK( ) ;
      //
      }
	return ;
} 



//*****************************************************************************
// Read EEPROM Data All
//
// Command 		Format
//					SB OP		Address 				Data
//						CODE	x8			x16		x8			x16		Comments
//-----------------------------------------------------------------------------
//	READ 			1 	10 	A6-A0 	A5-A0								Reads data stored 
//																				in memory, at
//																				specified address.
//
// Timer
//***************************************************************************** 
unsigned char EE93CXX_ReadByte( unsigned short adr )
{
	unsigned char value ;

  CS_EErom(HIGH) ;
  DO_EErom(HIGH) ;
  EE93CXX_SK( ) ;
  DO_EErom(HIGH) ;
  EE93CXX_SK( ) ;
  DO_EErom(LOW) ;
  EE93CXX_SK( ) ;
  
  EE93CXX_WriteAddr(adr) ;
  DO_EErom(LOW) ;
  
  value = EE93CXX_ReceiveByte() ;
  
  CS_EErom(LOW) ;
  SK_EErom(LOW) ;	
	 
	return value ;
}


//*****************************************************************************
// Recevice Data Byte
//*****************************************************************************
unsigned char EE93CXX_ReceiveByte( )
{ 
	unsigned char i;
	unsigned char value=0;

	for( i = 0;  i < 8 ;  i++ )
	  {  
		EE93CXX_SK( ) ;
		value = value << 1 ;
    value = value | DI_EErom() ;
	  }
	 
	return value;  
}



//***************************************************************************
// EEPROM SK CLOCK Sub-Program
// 
// AT93Cxx Specification
// Frequency	High		Low		voltage
// 	2MHz		250ns		250ns		5V
//		1MHz		500ns		500ns		3.3V
// 
// 
//			 +--------+			   +--------+
//			 |			  |			   |
// ------+			  +--------+
//			    ^						^
//			    |	500ns   500ns	|
//
//***************************************************************************
void EE93CXX_SK( )
{
  //
  SK_EErom(LOW) ;
	SK_EErom(HIGH) ;
  //
	return ;
}


/******************* (C) COPYRIGHT 2009 Johnson Fitness Inc. ***END OF FILE****/

