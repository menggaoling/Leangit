

#ifndef __LOADERUART_H__
#define __LOADERUART_H__



#define   _MaxDataLength_       1024
#define   _MaxBufferLength_     (_MaxDataLength_*2+10)


//
void LoaderUART_Initial(void) ;
void LoaderUART_TxRx_Information(void) ;
void LoaderUART_RxProcess(void) ;


#endif /* __LOADERUART_H__*/


