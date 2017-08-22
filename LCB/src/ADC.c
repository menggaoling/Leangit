//##############################################################################


//##############################################################################
/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "ADC.h"
#include  "LCBMain.h"
#include  "EMControl.h"
// 2013.12.03
//#include  "MultiMCUComm.h"
//
#include  "PinDefine.h"



extern LCBADValueStruct  LCBADC ;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address           ((uint32_t)0x4001244C)
#define ADC1_NumberOfChnnel       10     
#define ADC_FilterDataLength      5
#define ADC_FilterAvgSize         3
#define ADC_FilterAvgStart        ((ADC_FilterDataLength-ADC_FilterAvgSize)/2)

//#define _ENABLE_                  0x5A
//#define _DISABLE_                 0xA5
//#define _OFFSET_AVG_SIZE_         50
#define _ADC_AVG_SIZE_            10
#define ADC_OffsetDelay           10 
#define ADC_SampleTime            1


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile unsigned short ADC1ConvertedValue[ADC1_NumberOfChnnel] ;
ADCDataStruct ADCData[ADC1_NumberOfChnnel] ;
volatile unsigned short ADCValue[ADC1_NumberOfChnnel] ;
unsigned short ADCFilterData[ADC1_NumberOfChnnel][ADC_FilterDataLength] ;
//unsigned char EnableADCOffset ;
unsigned char FilterDataCounter ;
volatile ADCStatusReg ADCStatus ;
unsigned char OffsetDelayCounter ;
unsigned char ADC_SampleTimeCount ;

/* Private function prototypes -----------------------------------------------*/
void ADC_DataFIFO( unsigned short *ptr, unsigned short NewData,unsigned char Length ) ;
void ADC_FilterBubbleSort( unsigned short* iSource, unsigned char Length ) ;
unsigned short ADC_FilterAverage( unsigned short* iSource, unsigned char Length ) ;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Initial(void)
{
  ADC_InitTypeDef           ADC_InitStructure;
  DMA_InitTypeDef           DMA_InitStructure;
  
  //----------------------------------------------------------------------------
  unsigned char i ;
  for( i = 0; i < ADC1_NumberOfChnnel ;i++)
  {
      ADCData[i].max = 0 ;
      ADCData[i].min = 0 ;
      ADCData[i].count = 0 ;
      ADCData[i].sum = 0 ;
      ADCData[i].adverage = 0 ;
      ADCValue[i] = 0 ;
      ADCData[i].offsetADC = 0 ;
      ADC1ConvertedValue[i] = 0 ;
  }
  //EnableADCOffset = _ENABLE_ ;
  FilterDataCounter = 0 ;
  OffsetDelayCounter = 0 ;
  ADCStatus.All = 0 ;
  ADCStatus.B.SetOffset = 1 ;
  ADC_SampleTimeCount = 0 ;
  
  /* DMA1 channel1 configuration ---------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = ADC1_NumberOfChnnel ;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* Enable DMA1_Channel1 Interrupt */
  DMA_ITConfig( DMA1_Channel1,DMA_IT_TC,ENABLE ) ;

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = ADC1_NumberOfChnnel ;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10,1, ADC_SampleTime_239Cycles5); // Battery voltage  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11,2, ADC_SampleTime_239Cycles5); // Battery current
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12,3, ADC_SampleTime_239Cycles5); // Boost Current 升壓電路電流
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13,4, ADC_SampleTime_239Cycles5); // Generator Voltage
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14,5, ADC_SampleTime_239Cycles5); // ECB Current
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 6, ADC_SampleTime_239Cycles5); // Generator Current 預留
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_239Cycles5); // Main Power 12V (發電機所產生的電源)
  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 8, ADC_SampleTime_239Cycles5); // Boost Voltage 15V 15V充電電壓(升壓電路)
  ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 9, ADC_SampleTime_239Cycles5); // Resistance Offset
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15,10,ADC_SampleTime_239Cycles5); // ECB voltage //Temp Sensor
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable Vrefint channel17 */
  ADC_TempSensorVrefintCmd(ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
  
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  //
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ConversionProcess(void)
{
  unsigned char i ;
  unsigned short Temp ;
  // Calculator
  for( i = 0 ; i < ADC1_NumberOfChnnel ; i++ )
  {
      //  
      if( ADCData[i].count == 0 )
      {
          ADCData[i].min = ADC1ConvertedValue[i] ;
          ADCData[i].max = ADC1ConvertedValue[i] ;
      }
      else
      {
          if( ADCData[i].max < ADC1ConvertedValue[i] )
              ADCData[i].max = ADC1ConvertedValue[i] ;
          else
          {
              if( ADCData[i].min > ADC1ConvertedValue[i] )
                  ADCData[i].min = ADC1ConvertedValue[i] ;  
          }
      }
      //
      ADCData[i].count += 1 ;
      ADCData[i].sum += ADC1ConvertedValue[i] ;
      if( ADCData[i].count > _ADC_AVG_SIZE_ )
      {
          ADCData[i].adverage = ( ADCData[i].sum - ADCData[i].max - ADCData[i].min ) / (ADCData[i].count-2) ;
          //ADCValue[i] = ADCData[i].adverage ;
          ADCData[i].max = 0 ;
          ADCData[i].min = 0 ;
          ADCData[i].count = 0 ;
          ADCData[i].sum = 0 ;
          ADC_DataFIFO(&ADCFilterData[i][0],ADCData[i].adverage,ADC_FilterDataLength) ;
          if( i == (ADC1_NumberOfChnnel-1) )
          {
              FilterDataCounter += 1 ;    
          }
      }
  }    
  //----------------------------------------------------------------------------
  if( FilterDataCounter > (ADC_FilterDataLength-1) )
  {
      FilterDataCounter = 0 ;
      ADCStatus.B.ADCDataComplete = 1 ;
      for( i = 0 ; i < ADC1_NumberOfChnnel ; i++ )
      {
          ADC_FilterBubbleSort(&ADCFilterData[i][0],ADC_FilterDataLength) ;
          if( ADCStatus.B.SetOffset == 1 )
          {
              if( OffsetDelayCounter == 0 )
                  ADCData[i].offsetADC =  ADC_FilterAverage(&ADCFilterData[i][ADC_FilterAvgStart],ADC_FilterAvgSize) ;
              else
              {
                  Temp = (ADCData[i].offsetADC+ ADC_FilterAverage(&ADCFilterData[i][ADC_FilterAvgStart],ADC_FilterAvgSize)) /2 ;
                  ADCData[i].offsetADC = Temp ;
              }
          }
          else  
          {
              if( ADCValue[i] == 0 )
                  ADCValue[i] = ADC_FilterAverage(&ADCFilterData[i][ADC_FilterAvgStart],ADC_FilterAvgSize) ;
              else
              {
                  Temp = (ADCValue[i]+ ADC_FilterAverage(&ADCFilterData[i][ADC_FilterAvgStart],ADC_FilterAvgSize)) / 2 ;
                  ADCValue[i] = Temp ;  
              }
          }
      }
      // Check Set ADC Offset 
      if( ADCStatus.B.SetOffset == 1 )
      {
          OffsetDelayCounter += 1 ;
          if( OffsetDelayCounter > ADC_OffsetDelay )
          {
              ADCStatus.B.SetOffset = 0 ;
              OffsetDelayCounter = 0 ;
          }
          //
      }
  }
  //  
  /* Clear Channel 1 DMA1_FLAG_TC flag */
  DMA_ClearFlag(DMA1_FLAG_TC1);	
  /* Start ADC1 Software Conversion */ 
  //ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  //
  ADCStatus.B.ADCSampleComplete = 1 ;
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Process(void)
{
  if( ADCStatus.B.ADCDataComplete == 1 )
  {
      if( ADCStatus.B.SetOffset == 0 )
      {
          LCBADC.SecondDC12 = ADC_GetCalculatorValue(_MainPower12V_) ;
          LCBADC.ECBCurrent = ADC_GetCalculatorValue(_ECBCurrent_) ;
          LCBADC.BoostVoltage = ADC_GetCalculatorValue(_BoostVoltage15V_) ;  
          LCBADC.BoostCurrent = ADC_GetCalculatorValue(_BoostCurrent_) ;
          LCBADC.GeneratorCurrent = ADC_GetCalculatorValue(_GeneratorCurrent_) ;
          LCBADC.GeneratorVoltage = ADC_GetCalculatorValue(_GeneratorVoltage_)  ;
          LCBADC.BatteryChargeCurrnet = ADC_GetCalculatorValue(_BatteryCurrent_) ;
          LCBADC.BatteryVoltage = ADC_GetCalculatorValue(_BatteryVoltage_) ;
          LCBADC.ECBvoltage = ADC_GetCalculatorValue(_ECBvoltage_) ;        
                  
          ADCStatus.B.ADCDataSatndby = 1 ;
      }
      ADCStatus.B.ADCDataComplete = 0 ;
  }
  //----------------------------------------------------------------------------
  if( ADCStatus.B.ADCSampleComplete == 1 )
  {
      ADCStatus.B.ADCSampleComplete = 0 ;
      ADC_SampleTimeCount = 0 ;
      /* Start ADC1 Software Conversion */ 
      ADC_SoftwareStartConvCmd(ADC1, ENABLE);      
  }
  //----------------------------------------------------------------------------    
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          :    
*                  _BatteryVoltage_        0
*                  _BatteryCurrent_        1
*                  _BoostCurrent_          2
*                  _GeneratorVoltage_      3
*                  _ECBCurrent_            4
*                  _GeneratorCurrent_      5
*                  _MainPower12V_          6
*                  _BoostVoltage15V_       7
*                  _ResistanceOffset_      8
*                  _ECBvoltage_            9
* Output         : None
* Return         : None
*******************************************************************************/
unsigned short ADC_GetADC(unsigned char source)
{
  //
  if( source < 10 )
  {
      return ADCValue[source] ;
  }
  //
  return 0 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          :    
*                  _BatteryVoltage_        0
*                  _BatteryCurrent_        1
*                  _BoostCurrent_          2
*                  _GeneratorVoltage_      3
*                  _ECBCurrent_            4
*                  _GeneratorCurrent_      5
*                  _MainPower12V_          6
*                  _BoostVoltage15V_       7
*                  _ResistanceOffset_      8
*                  _ECBvoltage_            9
* Output         : None
* Return         : None
*******************************************************************************/
unsigned short ADC_GetCalculatorValue(unsigned char source)
{
  unsigned long Result = 0 ;
  switch( source )
  {
      case  _BatteryVoltage_  :
                              // 3V = 15.65V => 3.3V = 17.19V
                              Result = (long)ADCValue[_BatteryVoltage_] * 1719 / 4095 ;
                              break ;
      case  _BatteryCurrent_  :
                              // 3V = 3A => 3.3V = 3.3A 
                              Result = (long)ADCValue[_BatteryCurrent_] * 330 / 4095 ; 
                              break ;
      case  _BoostCurrent_    :
                              // 3V = 2.39 => 3.3V = 2.63
                              Result = (long)ADCValue[_BoostCurrent_] * 263 / 4095;
                              break ;
      case  _GeneratorVoltage_  :
                              // 3V = 300V => 3.3V = 330V
                              Result = (long)ADCValue[_GeneratorVoltage_] * 330 / 4095; 
                              break ;
      case  _ECBCurrent_  :
                              // 3V = 2A  => 3.3V = 2.2A 
                              if(EMControl_GetPWM() >= 1)  
                              {
                                  //Result = (long)ADCValue[_ECBCurrent_] * 2200 / 4095;  //2015.10.21
                                  if(ADCValue[_ECBCurrent_] < ADCData[_ECBCurrent_].offsetADC)
                                    Result = 0;
                                  else
                                    Result = (long)(ADCValue[_ECBCurrent_] - ADCData[_ECBCurrent_].offsetADC) * 2200 / 4095;
                                  Result = (long)Result * 10 / EMControl_GetPWM();
                                  if(EMControl_GetPWM() <= 5)  Result = Result * 5 / 10;
                                  else if(EMControl_GetPWM() <= 7) Result = Result * 7 / 10;
                                  else if(EMControl_GetPWM() <= 9) Result = Result * 8 / 10;
                                  else if(EMControl_GetPWM() <= 11) Result = (long)Result * 85 / 100;
                                  else if(EMControl_GetPWM() <= 13) Result = Result * 9 / 10;
                                  else if(EMControl_GetPWM() <= 15) Result = (long)Result * 95 / 100;
                              }
                              break ;
      case  _GeneratorCurrent_ :
                              // 3V = 3A => 3.3V = 3.3A 
                              Result = (long)ADCValue[_GeneratorCurrent_] * 330 / 4095;
                              break ;
      case  _MainPower12V_    :
                              // 3V = 15.1V => 3.3V = 16.61V
                              Result = (long)ADC1ConvertedValue[_MainPower12V_] * 1661 / 4095;
                              break ;
      case  _BoostVoltage15V_ :
                              // 3V = 17.1V => 3.3V = 18.81V
                              Result = (long)ADCValue[_BoostVoltage15V_] * 1881 / 4095;
                              break ;
      case  _ResistanceOffset_:
                              // 0~3.3V
                              Result = (long)ADCValue[_ResistanceOffset_] * 3300/ 4095 ;
                              break ;
      case  _ECBvoltage_      :
                              // 3V = 33V => 3.3V = 36.3V
                              Result = (long)ADCValue[_ECBvoltage_] * 3630 / 4095 ; 
                              break ;
  }
  //
  return (unsigned short)Result ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char ADC_GetDataSatndbyStatus(void)
{
  if( ADCStatus.B.ADCDataSatndby == 1 )
      return 1 ;
  return 0 ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_DataFIFO( unsigned short *ptr, unsigned short NewData,unsigned char Length )
{
  unsigned short *ptr_bottom ;
  unsigned char	i ;
  unsigned short TmpData ;
  //
  ptr_bottom = ptr ;
  for( i = (Length-1)  ; i > 0 ; i-- )
  {
      TmpData = ptr_bottom[i-1] ;
      ptr_bottom[i] = TmpData ;
  }
  ptr_bottom[0] = NewData ;	
  //
  return  ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_FilterBubbleSort( unsigned short* iSource, unsigned char Length ) 
{
  unsigned char x , y ;
  unsigned short Temp ;
  //
  if( Length <= 1 )
      return ;
  //
  for( x = 0 ; x < Length ; x++ )
  {
      for( y = (x+1) ; y < Length ; y++ )
      {
          if( iSource[x] > iSource[y] )
          {
              Temp = iSource[x] ;
              iSource[x] = iSource[y] ;
              iSource[y] = Temp ;
          }
      }
  }		
  //	
  return ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned short ADC_FilterAverage( unsigned short* iSource, unsigned char Length ) 
{
  unsigned long iAverage = 0;
  unsigned char i = 0;
  //---------------------------------------------------------------------------- 
  for( i = 0 ; i < Length ; i++ )
  {
      iAverage = iAverage + (unsigned long)iSource[i];
  }
  //----------------------------------------------------------------------------
  return ((unsigned short)(iAverage/Length));
}
/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

unsigned short ADC_ConverterEMCurrentToADC(unsigned short iA)
{
  unsigned short y;
  if(iA > 220) iA = 220;
  y = iA * 4095 / 220;
  return y;
}




