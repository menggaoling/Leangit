#include  "PinDefine.h"
#include  "JHTLCBComm.h"
#include  "EMControl.h"
#include  "LCBMain.h"
#include  "RPM.h"
#include  "ADC.h"

#define _ECB_CheckTime_               2000
#define _ECB_OVErrorTime_             (_ECB_CheckTime_/4)
#define _ECB_NCErrorTime_             (_ECB_CheckTime_/2)

union {
    struct {
        //unsigned short SystemTime:1 ;
        //unsigned short PowerOn:1 ;
        //unsigned short ResistanceEnable:1 ;
        //unsigned short ADCError:1 ;
        //unsigned short ResistanceOpen:1 ;
        //unsigned short ResistanceShort:1 ;
        unsigned short EM_CurrentUnder:1 ;
        unsigned short EM_CurrentOver:1 ;
        unsigned short Rev:14 ;
    } B ;
    unsigned short All ;
} ControlStatus ;


unsigned char EM_C_UnderTime ;
unsigned char EM_C_OverTime ;
unsigned char EM_C_HoldTime ;
unsigned short ECB_OverCurrentTime ;
unsigned short ECB_NoConnectTime ;
unsigned short TragetEM_ADC ;
unsigned short ControlCurrentCommand ;
unsigned short EMPWMValue  ;
unsigned short OldEMPWMValue ;
unsigned short PWMModeValue = 0 ;
unsigned short EM_TrackingTimeCount = 0 ;
unsigned short EM_OCTime =0  ;
unsigned short EM_NCTime = 0 ;
unsigned short TempTime = 0 ;
//unsigned short _CheckTimeOut = 0;
unsigned short Now_GV = 0;

void EMControl_ElectromagnetCurrentControl(void);


void EMControl_Initial(void)
{
    ControlStatus.All = 0 ;
}
// 控制
// 放置於 system tick 進行作業
void EMControl_Running(void)
{
    if( LCBMain_GetConsolePowerStatus() == 1 )//ControlStatus.B.PowerOn
    {   
        if( FeedBackAdcData.Status.B.EM_OverCurrent == 0 && 
           FeedBackAdcData.Status.B.EM_NoConnection ==0 )
        {
            //_CheckTimeOut++;
            if( SendData.EM_TestPWM == 0 )
            {
                if( PWMModeValue != SendData.EM_TestPWM )
                {
                    PWMModeValue = 0 ;
                    EcbOFF;  
                }
                EMControl_ElectromagnetCurrentControl() ;// EC控制副程式
            }
            else
            {
                EMPWMValue = 0 ;
                if( SendData.EM_CurrentCommand == 0 )
                {
                    if( PWMModeValue != SendData.EM_TestPWM )
                    {
                        PWMModeValue = SendData.EM_TestPWM ;
                        if( PWMModeValue > EcbMaxLimit ) 
                            PWMModeValue = EcbMaxLimit ; 
                        EcbPWM(PWMModeValue);
                    }
                }
                else
                    EcbPWM(EMPWMValue);
            }
            //----------------------------------------------------------------
            // Check ECB Current
            if( ADC_GetCalculatorValue(_ECBCurrent_) > 210 )//FeedBackAdcData.ElectroMagnetCurrent
            {
                ECB_OverCurrentTime += 1 ;
                if( ADC_GetCalculatorValue(_ECBCurrent_) > 250 )//FeedBackAdcData.ElectroMagnetCurrent
                    ECB_OverCurrentTime += 1 ;
                ECB_NoConnectTime = 0 ;
                if( ECB_OverCurrentTime >= 2000 )
                {
                    ECB_OverCurrentTime = 0 ;
                    FeedBackAdcData.Status.B.EM_OverCurrent = 1 ;
                    EcbOFF;  
                }
            }
            else
            {
                ECB_OverCurrentTime = 0 ;
                if( SendData.EM_CurrentCommand > 50 && 
                   ADC_GetCalculatorValue(_GeneratorVoltage_) > SP_DCBUS_LIMIT && 
                       (RPMData.AverageRPM >= SP_CONSOLE_POWER_ON_RPM + 15) && 
                           ADC_GetCalculatorValue(_ECBCurrent_) < 5 )//FeedBackAdcData.GeneratorVoltage  //FeedBackAdcData.ElectroMagnetCurrent
                {
                    ECB_NoConnectTime += 1 ;
                    if( ECB_NoConnectTime >= 5000 )
                    {
                        ECB_NoConnectTime = 0 ;
                        FeedBackAdcData.Status.B.EM_NoConnection = 1 ;
                        EcbOFF;  
                    }
                }
                else
                    ECB_NoConnectTime = 0 ;
            }
        }
        else
        {
            EcbOFF;    
            ECB_NoConnectTime = 0 ;
            ECB_OverCurrentTime = 0 ;
        }
    }
}
//==============================================================================
// EC控制副程式
//==============================================================================
void EMControl_ElectromagnetCurrentControl(void)
{
    unsigned short EM_C = 0;
    unsigned short PWMOffset = 3;   // 0.125%
    static unsigned char reCtlFlg = 1;
    // Check Refresh  
    if( ++EM_TrackingTimeCount >= 200 )
    {
        EM_TrackingTimeCount = 0 ;
        Now_GV = ADC_GetCalculatorValue(_GeneratorVoltage_);
        ControlCurrentCommand = SendData.EM_CurrentCommand;
        
        if(Now_GV < EM_OFF_DCBUS_MIN + 20)
        {         
            if((RPMData.AverageRPM >= 50 && Now_GV < 45)
               || (RPMData.AverageRPM >= 45 && Now_GV <= 40) || (RPMData.AverageRPM >= 40 && Now_GV <= 35)
                   || (RPMData.AverageRPM >= 35 && Now_GV <= 30) || (RPMData.AverageRPM >= 25 && Now_GV < 25))
            {   
                if(EMPWMValue >= 1200)
                    EMPWMValue -= 216;     //9%
                else if(EMPWMValue >= 960)
                    EMPWMValue -= 168;     //7%
                else if(EMPWMValue >= 720)
                    EMPWMValue -= 120;     //5%               
                else 
                {
                    if(EMPWMValue >= 72) 
                        EMPWMValue -= 72;  //3%
                }
                EcbPWM(EMPWMValue);
                return;
            }                 
        }
        
        if( ControlCurrentCommand < 5 || RPMData.AverageRPM < SP_CONSOLE_POWER_ON_RPM || Now_GV < EM_OFF_DCBUS_MIN) 
        {
            if(ControlCurrentCommand >= 5) 
            {
                reCtlFlg = 0;
            }
            EcbOFF; 
            EMPWMValue = 0;
            return;
        }
        
        if((RPMData.AverageRPM > SP_CONSOLE_POWER_ON_RPM + 4) && ADC_GetCalculatorValue(_GeneratorVoltage_) > EM_OFF_DCBUS_MIN + 4 ) reCtlFlg = 1;
        
        if(reCtlFlg == 1)
        {
            ControlStatus.B.EM_CurrentUnder = 0 ;
            ControlStatus.B.EM_CurrentOver = 0  ;  
            if( ADC_GetCalculatorValue(_ECBCurrent_) > ControlCurrentCommand ) 
            {
                ControlStatus.B.EM_CurrentOver = 1  ;
                EM_C = ADC_GetCalculatorValue(_ECBCurrent_) - ControlCurrentCommand ; 
            }
            else if( ADC_GetCalculatorValue(_ECBCurrent_) < ControlCurrentCommand )
            {
                ControlStatus.B.EM_CurrentUnder = 1 ;
                EM_C = ControlCurrentCommand - ADC_GetCalculatorValue(_ECBCurrent_) ;
            }
            
            if( EM_C > 28 )
                PWMOffset = 168;      // 7%             
            else if( EM_C > 17 )
                PWMOffset = 120;      // 5%         
            else if( EM_C > 9 )
                PWMOffset = 72;       // 3%
            else if( EM_C > 3 )
                PWMOffset = 24;       // 1%  
            else if( EM_C > 1 )
                PWMOffset = 6;        // 0.25%  
            
            if( ControlStatus.B.EM_CurrentOver == 1 )
            {          
                if( EMPWMValue >= PWMOffset )
                    EMPWMValue -= PWMOffset ;     
            }
            else if( ControlStatus.B.EM_CurrentUnder == 1 )
            {
                EMPWMValue += PWMOffset ;
                if( EMPWMValue > _PWM_MAX_ ) 
                    EMPWMValue = _PWM_MAX_ ; 
            }                  
        }
        
        EcbPWM(EMPWMValue);
    }
}

unsigned short EMControl_GetPWM(void)
{ 
    return  (unsigned long)EMPWMValue * 100 / EcbMaxLimit;
}


