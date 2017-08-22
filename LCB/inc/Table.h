#ifndef __TABLE_H__
#define __TABLE_H__


unsigned short Table_GetElectromagnetCurrent(unsigned short Watts,unsigned short Rpm) ;
unsigned short Table_GeneratorCurrent2Watts(unsigned short Current,unsigned short Rpm) ;
unsigned short Table_ECBCurrent2Watts(unsigned short Current,unsigned short Rpm) ;
unsigned short Table_LCBBassWatts(unsigned short Rpm) ;



#endif /* __TABLE_H__ */


