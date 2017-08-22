

#ifndef __BOOTUART_H__
#define __BOOTUART_H__




#define   _MaxDataLength_       50
#define   _MaxBufferLength_     (_MaxDataLength_+10)
#define   _RXD              0
#define   _TXD              1

//
void Bootuart_Initial(void) ;
void Bootuart_TxRx_Information(void) ;
void Bootuart_RxProcess(void) ;
void Bootuart_EnableRx(unsigned char Mode) ;



#endif /* __BOOTUART_H__*/


