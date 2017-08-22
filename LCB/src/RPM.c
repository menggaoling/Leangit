/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "RPM.h"
#include  "LCBMain.h"

// A2 Change TIM1 to TIM4

extern LCBSystemControlDataStatus  LCBEeprom ;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define   _RPM_MAX_SIZE_          6
#define   _TIM_DIV_               100
#define   _TIM_BASE_Freq_         (SystemCoreClock/_TIM_DIV_) // 48000000 / 100
unsigned short RPMFrequency[_RPM_MAX_SIZE_] ;
unsigned short RpmCaptureTimeout ;
unsigned char  RpmCaptureCount ;

RPMDataStruct RPMData ;

/* Private function prototypes -----------------------------------------------*/
void RPM_FilterRPMFIFO( unsigned short *ptr, unsigned short NewData , unsigned char Length ) ;
unsigned short RPM_FilterRPMAverage( unsigned short* iSource, unsigned char Length )  ;

/* Private functions ---------------------------------------------------------*/
void RPM_Initial(void)
{
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    TIM_ICInitTypeDef         TIM_ICInitStructure;
    unsigned char i ;
    //
    for( i = 0 ; i < _RPM_MAX_SIZE_ ; i++ )
        RPMFrequency[i] = 0 ;
    RpmCaptureTimeout = 0 ;
    RPMData.AverageRPM = 0 ;
    RPMData.GearRate = &LCBEeprom.Member.GearRate ;
    RPMData.GenMegPolePair = &LCBEeprom.Member.GenMegPolePair ;
    RPMData.Frequency = 0 ;
    RpmCaptureCount = 0;
    //
    /* -----------------------------------------------------------------------
    TIM4 Configuration: 
    TIM4CLK = 48 MHz, Prescaler = 100, TIM4 counter clock = 480KHz 
    */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF ;
    TIM_TimeBaseStructure.TIM_Prescaler = (_TIM_DIV_-1) ;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    /* TIM4 configuration: PWM Input mode ------------------------
    The external signal is connected to TIM4 CH1 pin (PB.06), 
    The Rising edge is used as active edge,
    The TIM4 CCR2 is used to compute the frequency value 
    The TIM4 CCR1 is used to compute the duty cycle value
    ------------------------------------------------------------ */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 ;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV4;
    TIM_ICInitStructure.TIM_ICFilter = 0x4;
    TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);
    /* Select the TIM4 Input Trigger: TI1FP1 */
    TIM_SelectInputTrigger(TIM4,TIM_TS_TI1FP1 );
    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
    /* Enable the Master/Slave Mode */
    TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
    /* TIM enable counter */
    TIM_Cmd(TIM4, ENABLE);
    /* Enable the CC1 Interrupt Request */
    TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
    //----------------------------------------------------------------------------
    return ;
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RPM_CaptureInput(void) 
{
    unsigned short IC1Value,IC2Value ;
    unsigned char DutyCycle = 0;
    static unsigned short OldRPM = 0 ; 
    static unsigned char FilterBigCnt = 0 ;
    static unsigned char FilterSmallCnt = 0;
    volatile unsigned short TempRPM  = 0 ;
    //
    if(TIM_GetITStatus(TIM4,TIM_IT_CC1 ) == SET)
    {
        /* Clear TIM4 Capture compare interrupt pending bit */
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        /* Get the Input Capture value */
        IC2Value = TIM_GetCapture1(TIM4);
        IC1Value = TIM_GetCapture2(TIM4);
        if( IC2Value != 0 )
        {
            if( ++RpmCaptureCount > 5 )
            {
                RpmCaptureCount = 5 ;
                /* Duty cycle computation (百分比)*/
                DutyCycle = (IC1Value* 100) / IC2Value;
                /* Frequency computation */
                RPMData.Frequency = (float)_TIM_BASE_Freq_ / IC2Value;
                //
                if( DutyCycle < 70 )
                {
                    // 6000為轉分鐘所以乘60,然後因為速比的關係所以須在補償回來乘100.(速比原先為4.8但我們放大為480) 
                    TempRPM = (unsigned short)( RPMData.Frequency * 6000 / *RPMData.GenMegPolePair / *RPMData.GearRate) ;
                    
                    if(TempRPM + 5 < OldRPM)
                    {
                        FilterBigCnt = 0;
                        if(++FilterSmallCnt > 3)
                        {
                            FilterSmallCnt = 0;
                            OldRPM = TempRPM;
                            RPM_FilterRPMFIFO( RPMFrequency, TempRPM , _RPM_MAX_SIZE_ ) ;
                        }
                    }    
                    else if(TempRPM > OldRPM + 5)
                    {
                        FilterSmallCnt = 0;
                        if(++FilterBigCnt > 3)
                        {
                            FilterBigCnt = 0;
                            OldRPM = TempRPM;
                            RPM_FilterRPMFIFO( RPMFrequency, TempRPM , _RPM_MAX_SIZE_ ) ;
                        }
                    } 
                    else
                    {
                        FilterBigCnt = 0;
                        FilterSmallCnt = 0;
                        OldRPM = TempRPM;
                        RPM_FilterRPMFIFO( RPMFrequency, TempRPM , _RPM_MAX_SIZE_ ) ; 
                    }
                    
                    RpmCaptureTimeout = 2000 ;
                    //
                }
            }
            //
        }
        else
        {
            FilterBigCnt = 0;
            FilterSmallCnt = 0;
        }
    }
    return ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RPM_CaptureTimeoutProcess(void)
{
    if( RpmCaptureTimeout > 0 )
    {
        --RpmCaptureTimeout;
        if(RpmCaptureTimeout == 0)
        {
            RPMData.AverageRPM = 0 ;
            RPMData.Frequency = 0 ;
            for( unsigned char i = 0 ; i < _RPM_MAX_SIZE_ ; i++ )
                RPMFrequency[i] = 0 ;
        }
    }
    return ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RPM_FilterRPMFIFO( unsigned short *ptr, unsigned short NewData , unsigned char Length ) 
{
    unsigned short *ptr_bottom ;
    unsigned char	i ;
    unsigned short TmpData ;
	//----------------------------------------------------------------------------
    //if( NewData < 240 )
    //    {
    ptr_bottom = ptr ;
    for( i = (Length-1)  ; i > 0 ; i-- )
    {
        TmpData = ptr_bottom[i-1] ;
        ptr_bottom[i] = TmpData ;
    }
    ptr_bottom[0] = NewData ;	
    //    }
    //----------------------------------------------------------------------------
    return  ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned short RPM_CalculatorAnalogRPM(void)
{
    RPMData.AverageRPM = RPM_FilterRPMAverage( RPMFrequency , _RPM_MAX_SIZE_ ) ;
    if(RPMData.AverageRPM < 10) RPMData.AverageRPM = 0;
    return RPMData.AverageRPM ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned short RPM_FilterRPMAverage( unsigned short* iSource, unsigned char Length ) 
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


