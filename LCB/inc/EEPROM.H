//##############################################################################
//
//##############################################################################


#define EE_PARAMETER_SPACE       1
#define EE_ERROR_REPORT_SPACE    2  
#define EE_GE_V_REPORT           3 
#define EE_GE_A_REPORT           4 
#define EE_BAT_A_REPORT          5 
#define EE_EM_A_REPORT           6 
#define EE_TRACKING_REPORT_SPACE 7
#define EE_Block                 8

#define EE_ParameterMax 8

#define C_FAIL      0
#define C_PASS      1
// Flash °Ï¶ô¬O§_±Ò¥Î
#define C_FlashON  0xE0
#define C_FlashOFF 0xE1
//

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
//unsigned char EEPROM_CheckParamter(void) ;
unsigned char EEPROM_Initial(void) ;
void EEPROM_SaveParameter(char by_D) ;
void EEPROM_ErrorReportInitial(void) ;
unsigned char EEPROM_CheckDataBaseVersion(void) ;
//void EEPROM_DefaultDataBaseValue(void) ;
unsigned char EEPROM_GetErrorReportIndex(void) ;
unsigned char EEPROM_CheckSumCalculator(unsigned char *pData, unsigned char Length) ; 	
void EEPROM_ErrorReportSave(void) ;
void EEPROM_InitialMaxReport(void) ;
unsigned char EEPROM_MaxDataReportSave(unsigned char Index ) ;
unsigned char EEPROM_GetRealTimeReportIndex(void) ;
void EEPROM_InitialRealTimeReportIndex(void);
//unsigned char EEPROM_SaveRealTimeReportData(unsigned char NumberIndex) ;
unsigned char EEPROM_ReadInformation(unsigned char by_Address,unsigned char by_D);
void EEPROM_InitialParameter_C(void);
unsigned char EEPROM_WriteParameter_C(unsigned char NumberIndex);
void EEPROM_InitialParameter_B(void);
//------------------------------------------------------------------------------

#endif /* __EEPROM_H */






//===========================================================================
// End of file.
//===========================================================================
