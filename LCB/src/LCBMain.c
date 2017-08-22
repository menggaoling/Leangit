
//#define   DebugMonitor            1
//#define   DebugEMPWM              1
//#define     _DisableBoostOverCurrent          1
//#define   _EnableBatteryControlResistance     1

/* Includes ------------------------------------------------------------------*/
#include  "stm32f10x_conf.h"
#include  "RPM.h"
#include  "ADC.h"
#include  "PinDefine.h"
#include  "JHTLCBComm.h"
#include  "JHTCOMMAND.H"
#include  "BootUartCommand.h"
//#include  "EE93CXX.H"
#include  "EEPROM.h"
#include  "LCBMain.h"
#include  "Table.h"
#include  "EMControl.h"

//
extern RPMDataStruct RPMData ;
extern ErrorStatusDataStruct ErrorCodeStatus ;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 
#define   SP_BATTERY_POWER_RPM      25  
#define   SP_BATTERY_POWER_OFF_RPM  30
#define   SP_MAINPOWER_LIMIT        1100 


#define   MRESET                    50
#define   SELFPOWER_ON              (MRESET+1)
#define   DCPOWER_ON                (SELFPOWER_ON+1)
#define   BURNIN_TEST               (DCPOWER_ON+1)
#define   NORMAL                    (BURNIN_TEST+1)
#define   POWER_SAVE                (NORMAL+1)
#define   MAIN_POWER_OK             (POWER_SAVE+1)
#define   CONSOLE_POWER             (MAIN_POWER_OK+1)
#define   WAKEUP_CHECK              (CONSOLE_POWER+1)
#define   WAKEUP_LCB                (WAKEUP_CHECK+1)
#define   WAKEUP_UCB                (WAKEUP_LCB+1)
#define   WAIT_UCB_WAKEUP           (WAKEUP_UCB+1)  
#define   SLEEP_MODE                (WAIT_UCB_WAKEUP+1)  
#define   SLEEP_MODE1               (SLEEP_MODE+1)                 
#define   WAKEUP_FRPM               (SLEEP_MODE1+1)
//
#define   CHECK_MAIN_POWER          10
#define   CHECK_BOOST_HW            (CHECK_MAIN_POWER+1)
#define   CHECK_BATTERY_REV         (CHECK_BOOST_HW+1)
#define   CHECK_BATTERY_HW          (CHECK_BATTERY_REV+1)
#define   CHECK_DCPLUGIN            (CHECK_BATTERY_HW+1)
#define   CHECK_ECB                 (WAKEUP_FRPM+1)

#define   BOOST_HW_OK             10
#define   BOOST_HW_ERR            20
#define   BATTERY_HW_OK           30
#define   BATTERY_HW_ERR          40

#define   BoostModeCV             0
#define   BoostModeBCC            1
#define   BoostModeLBCC           2

#define   _DCpowerOnStableTime                  1000 
#define   _ACpowerOnStableTime                  3000 
#define   _UCBwakeupLCBwaitTime                 5000
#define   _LCBwakeupUCBstableTime               3000
#define   _WakeupCheckTime                      2000

#define   _SaveEepromTime                       10          // 10ms
#define   _UpdateLogFileTime                    1000        // 1000ms.
#define   _BoostOCLimtedTime                    300         // 300ms
#define   _BoostOCLimitedValue                  50          // 0.5A
#define   _BoostMaxLimitedValue                 40
#define   _MultiMCU_RxTimeoutLimit              1500        // 0: OFF ,


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile unsigned short SystemTimeCounter ;
volatile unsigned short CheckSaveEEPROMTime ;
volatile unsigned short UpdateLogFileTime ;
volatile unsigned short LogFileSaveTime ;

volatile unsigned char  LCBMainMode ;
volatile unsigned char  LCBSubMode ;
unsigned short StatusLED1Time ;
unsigned short StatusLED1TimeCounter ;
unsigned long  BurninTestTime ;
unsigned short BurninTestEM ;
unsigned short BurnTestPinLoTime ;
unsigned short BurnTestPinHiTime ;

LCBADValueStruct LCBADC ;
UCBDataStruct UCBData ;
LCBSystemControlDataStatus  LCBEeprom ;
volatile EEPROMSaveCtrDataType EepromControl ;
//
unsigned short DCPluginTestTime ;
unsigned short DCunPluginTestTime ;
unsigned short BatteryReverseTime ;
unsigned short BatteryNormalTime ;


// Boost Controller
unsigned short BoostOutputVoltage ;
unsigned short BoostControllerTime ;
unsigned short BoostPWMValue ;
volatile union {
    struct {
        unsigned char HardwareOK:1 ;
        unsigned char Enable:1 ;                    // Boost Controller Enable
        unsigned char VoltageInRange:1 ;
        unsigned char OC:1 ;
        unsigned char ControlMode:3 ;               // Control Voltage Mode
    } B ;
    unsigned char All ;
} BoostControllerStatus ;
unsigned short BoostControlOCTime ; 
//
volatile unsigned short MainPowerStableTime ;         // 20130617
unsigned short MainPowerStableTimeLimit ;    // 20130617
unsigned short CheckRPMTime ;
unsigned short BatteryDelayTime ;
unsigned long  BatteryKeepTime ;
unsigned long  BatteryKeepTimeCounter ;
unsigned short BatterySkipKeepTime ;
unsigned short BatteryChargeCheckTime ;
unsigned short BatteryFloatChargeCheckTime ;
unsigned short BatteryChargeCurrentLimit ;
unsigned short BatteryChargeHVTC ;
unsigned short BatteryChargeFLTC ;
unsigned short BatteryOCTimeCounter ;
unsigned short BatteryLCTimeCounter ;
unsigned short BatteryLowVoltageOnTC ;
unsigned short BatteryLowVoltageOffTC ;
unsigned short BatteryLVChargeTime ;
//
volatile union {
    struct {
        unsigned long sResetOK:1 ;                  // LCB Power On Reset OK
        unsigned long sBatteryChargeEnable:1 ;      // LCB Battrey Charge function Enable
        unsigned long sBatteryCharge:1 ;            // LCB Battery Charge
        unsigned long sBatteryDischarge:1 ;         // LCB Battery Discharge
        unsigned long sForceBatteryCharge:1 ;       // LCB 
        unsigned long sBatteryFloatCharge:1 ;    
        unsigned long sBatteryChargeSP:1 ;
        unsigned long sErPAction:1 ;                // LCB ErP Action
        unsigned long sKeepTimeAction:1 ;           // LCB Battery Keep Console Power
        unsigned long sWaitBatteryOnHelpPower:1 ;   // LCB Delay the Battery to Supply Console Power 
        unsigned long sWaitBatteryOffHelpPower:1 ;  // LCB Delay the Battery to Supply Console Power
        unsigned long sCheckDCPlugin:1 ;
        unsigned long sCheckDCPluginOK:1 ;
        unsigned long sWaitConsolePowerOn:1 ;
        unsigned long sBurninCheck:1 ;
        unsigned long sBurninCheckOK:1 ;
        unsigned long sCheckBatteryReverse:1 ;
        unsigned long sCheckBatteryReverseOK:1 ;
        //
        unsigned long sBatteryOK:1 ;                // LCB Battery Function OK
        unsigned long sBatteryConnect:1 ;           // LCB Battery Connecttion Relay ON
        unsigned long sBatteryLow:1 ;               // LCB Battery Low Voltage
        unsigned long sBatteryReverse:1 ;           // LCB Battery connection reverse
        unsigned long sDCPlugin:1 ;                 // LCB DC Plugin
        unsigned long sConsolePowerOn:1 ;           // LCB Power on for the console
        unsigned long sBurninTest:1 ;               // LCB Burn in test hardware action
        unsigned long sBurninTestRun:1 ;            // LCB Burn in running
        unsigned long sBatteryForMCU:1 ;            // LCB Switch Status (Battery for Mcu)
        unsigned long sPowerSourceChangeToSP:1 ;    // LCB DC -> SP
        unsigned long sPowerSourceChangeToDC:1 ;    // LCB SP -> DC
        unsigned long sCutOffResistance:1 ;         // LCB Class Error to Cut Off Resistance
        unsigned long sUpgradeMCU:1 ;               // LCB upgrade firmware
        //
    } B ;
    unsigned long All ;
} LCBSystemProcessStatus ;


unsigned short SendTestPWMTime ;
#ifdef  DebugMonitor
//unsigned short DebugADCOutTime ;
#endif
unsigned char LogMaxDataIndex;
unsigned char LogDataIndex;
unsigned char LogDataSaveTime;
#define   _LogDataSaveTimeLimit                 60
#define   _LogDataMaxIndex                      35

//const unsigned short  MACHINE_CW[24] = {50 ,	60 ,	72 ,	82 ,	95 ,	108 ,	119 ,	133 ,	144 ,	158 ,	170 ,	183 ,	199 ,	211 ,	228 ,	240 ,	254 ,	270 ,	285 ,	300 ,	315 ,	340 ,	354 ,	371};
const unsigned short  MACHINE_CW[24] = {36 ,	44 ,	53 ,	62 ,	72 ,	82 ,	92 ,	101 ,	112 ,	123 ,	134 ,	145 ,	158 ,	169 ,	182 ,	194 ,	207 ,	220 ,	232 ,	246 ,	260 ,	275 ,	288 ,	301};
const unsigned short MAX_WATT[24] = {500 ,770 ,1150 ,1520 ,1890 ,2250 ,2580 ,2900,3180 ,3460 ,3740 ,4020 ,4310 ,4600 ,4900 ,5200 ,5480 ,5750 ,6040 ,6320 ,6620 ,6920 ,7230 ,7540};

/* Private function prototypes -----------------------------------------------*/
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void GPIO_Reset(void) ;
void LCBMain_BurninCheck(void) ;
void LCBMain_DCPluginCheck(void) ;
unsigned char LCBMain_BoostHardwareCheck(void) ;
void LCBMain_BoostController(void) ;
unsigned char LCBMain_BatteryHardwareCheck(void) ;
void LCBMain_ConsolePowerControl(void) ;
void LCBMain_BatteryChargeControl(void) ;
void LCBMain_Resistance(void) ;
// 2013.12.03 
void LCBMain_EEPROM_Process(void);
//
/* Private functions ---------------------------------------------------------*/
unsigned char _BoostHWcheck = 0;

//------------------------------------------------------------------------------
void System_Initial(void)
{
    /* System Clocks Configuration */
    SystemInit() ;	
    
    /* Enable GPIOC clock */
    /*要打開外部中斷需要加上 RCC_APB2Periph_AFIO 設定*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | 
                           RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | 
                               RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | 
                           RCC_APB1Periph_TIM4 ,ENABLE );// 2014.02.25
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //
    // GPIO
    GPIO_Configuration() ;
    //----------------------------------------------------------------------------
    // Initial Parameter
    StatusLED1Time = 1000 ;
    StatusLED1TimeCounter = 0 ;
    CheckSaveEEPROMTime = 0 ;
    
    // Burnin Test Initial
    BurninTestEM = 0 ;
    BurninTestTime = 0 ;
    BurnTestPinLoTime = 0 ;
    BurnTestPinHiTime = 0 ;
    LCBMainMode = MRESET  ;
    DCPluginTestTime = 0 ;
    DCunPluginTestTime = 0 ;
    BatteryReverseTime = 0 ;
    BatteryNormalTime = 0 ;
    //
    BoostOutputVoltage = 0 ;
    BoostControllerTime = 0 ;
    BoostPWMValue = 0 ;
    BoostControllerStatus.All = 0 ;
    //
    LCBSystemProcessStatus.All = 0 ;
    SystemTimeCounter = 0 ;
    CheckRPMTime = 0 ;
    MainPowerStableTime = 0 ;
    MainPowerStableTimeLimit = _ACpowerOnStableTime ;
    BatteryDelayTime = 0 ;
    //BatteryKeepTime = 0 ;
    BatteryKeepTime = 30000 ;
    BatteryKeepTimeCounter = 0 ;
    BatterySkipKeepTime = 0 ;
    BatteryChargeCheckTime = 0 ;
    BatteryChargeCurrentLimit = 0 ;  
    BatteryChargeHVTC = 0 ;
    BatteryFloatChargeCheckTime = 0 ;
    BatteryOCTimeCounter = 0 ;
    BatteryLCTimeCounter = 0 ;
    BatteryChargeFLTC = 0 ;
    BatteryLowVoltageOnTC = 0 ;
    BatteryLowVoltageOffTC = 0 ;
    BatteryLVChargeTime = 0 ;
    EMControl_Initial();
    //----------------------------------------------------------------------------
    // PWM Input Capture
    RPM_Initial() ;
    //----------------------------------------------------------------------------
    // ADC
    ADC_Initial() ;
    //
    // UART
    JHTLCBComm_Initial() ;
    JHTLCBComm_HW_Initial() ;
    /* Setup SysTick Timer for 1 msec interrupts  */
    SysTick_Config(SystemCoreClock / 1000);
    
    /* NVIC Configuration */
    NVIC_Configuration();  
    //
    // Check EEPROM Parametar
    EepromControl.All = 0 ;
    // 2014.02.25 
    // EEPROM功能先行取消,因硬體不打件
    if(EEPROM_Initial() == C_FAIL)
    {//==>先判斷記憶體是否有跑掉
        //EEPROM_ErrorReportInitial();
    }
    else
    {
        EEPROM_ErrorReportInitial();
        LCBEeprom.Member.PowerOffTime = 1 ;  
        LCBEeprom.Member.ChargeCurrnetForNoResistance = 45 ;
    } 
    
    /*
    EEPROM_ErrorReportInitial();
    if( EEPROM_CheckParamter() == 0 )
    {
    //
    LCBEeprom.Member.RecordSize = _PARAMETER_NUMBER ;
    LCBEeprom.Member.Version = _EEPROM_VERSION ;      
    LCBEeprom.Member.ResistanceType.Full = 0x0087 ; // EM,13.5ohm
    LCBEeprom.Member.LimitRpmForResistance = 25 ; //
    LCBEeprom.Member.LimitRpmForCharge = 25 ; //
    // Fixed Parameter Default
    LCBEeprom.Member.PowerOffTime = 1 ;  
    LCBEeprom.Member.GenMegPolePair = 4 ;// 極數
    LCBEeprom.Member.MachineType = _BikeEP_ ;
    LCBEeprom.Member.GearRate = _DefaultGearRate ; // 速比 840
    LCBEeprom.Member.ChargeCurrnetForNoResistance = 45 ;
    // 
    EepromControl.B.SaveParameter = 1 ;
}
  else
    {
    LCBEeprom.Member.PowerOffTime = 1 ;  
    LCBEeprom.Member.ChargeCurrnetForNoResistance = 45 ;
}
    */
    //
    return ;      
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef 		NVIC_InitStructure;
    
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    //
    /* Enable the TIM4 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;//TIM1_CC_IRQn ; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the DMA1 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //
    //----------------------------------------------------------------------------
    // Enable the USART2 Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//Channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
    //
    //----------------------------------------------------------------------------
    return ;	
}



/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures GPIO.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  GPIO_Configuration(void)
{
    GPIO_InitTypeDef 	    GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
    //
    // TIM3 Must be Remap ( CH1/CH2/CH3/CH4 )
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE) ;
    // 2014.02.25
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE) ;
    // 2014.02.18
    if( LCBSystemProcessStatus.B.sErPAction == 0 )
    {
        GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_RESET) ;
        GPIO_WriteBit(GPIOA,GPIO_Pin_8,Bit_SET) ;
        GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET) ;
        GPIO_WriteBit(GPIOA,GPIO_Pin_10,Bit_SET) ;
        GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_SET) ;
        GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_SET) ;
        //
        GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET) ;
        GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_SET) ;
        GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET) ;
        GPIO_WriteBit(GPIOB,GPIO_Pin_14,Bit_SET) ;
        GPIO_WriteBit(GPIOB,GPIO_Pin_15,Bit_SET) ;  
        //
        GPIO_WriteBit(GPIOC,GPIO_Pin_10,Bit_SET) ;
        GPIO_WriteBit(GPIOC,GPIO_Pin_11,Bit_SET) ;
        GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET) ;
        //
        GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET) ;
    }
    // 2014.02.18
    // PA0 : ERP WKUP (RPM)
    // PA1 : Output, RS485 DE/RE
    // PA2 : Alternate Function Output, USART2(TX), RS485 TX
    // PA3 : Input Floating, USART2(RX), RS485 RX
    // PA4 : Analog Input 4, Generator Current 預留
    // PA5 : Analog Input 5, Power 12V
    // PA6 : Analog Input 6, Power 15V
    // PA7 : Analog Input 7, Resistance Adj(Resis_adj)
    // PA8 : Output, Status LED(LED22)
    // PA9 : Output, Status LED(LED23)
    // PA10: Output, Status LED(LED24)
    // PA11: Output, External Power Switch(EX_POWER_SW) 外部電源開關
    // PA12: Output, VCC Switch(VCC_SW) ERP電源開關
    //
    /********************* Configure  UART *******************/
    /* Configure USART1 / USART2 Rx (PA.10/PA3) as input floating */ 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;// UART2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure USART1 / USART2 Tx (PA.09/PA2) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;// UART2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Analog Intput
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    // Input Floating ( ERP RPM Capture )
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 2014.02.18
    // PB0 : Input Floating, 外部電源偵測
    // PB1 : NA
    // PB2 : NA
    // PB5 : Output, Battery Output(BAT_POWER) 電池電源開關
    // PB6 : Timer Capture Intput, RPM 
    // PB7 : NA
    // PB8 : NA
    // PB9 : NA
    // PB10: Output, ECB PWM ECB阻力控制 
    // PB11: Input Floating, DC Plugin(DC_IN) 
    // PB12: Output,EEPROM DI(93C46_DI)保留
    // PB13: Input Floating,EEPROM DO(93C46_DO)保留 
    // PB14: Output,EEPROM CLOCK(93C46_SK)保留
    // PB15: Output,EEPROM Chip Select(93C46_CS)保留
    // Input Floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6 | GPIO_Pin_11 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // PWM Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 2014.02.18  
    // PC0 : Analog Input 10, Battery Voltage(BAT_V)
    // PC1 : Analog Input 11, Battery Current(BAT_A)
    // PC2 : Analog Input 12, Boost Current(C1_A)
    // PC3 : Analog Input 13, Generator Current(GE_A)
    // PC4 : Analog Input 14, ECB Current(E_A_ADC)
    // PC5 : Analog Input 15, ECB Voltage(E_V_ADC)
    // PC6 : Alternate Function PWM Output, Battery Charge(Charge)
    // PC7 : Alternate Function PWM Output, Boost Voltage Control(Charge_V)
    // PC8 : NA
    // PC9 : Output, Status LED(LED21)
    // PC10: Output, Console Power Source Control(CON_P_12V)電源選擇(Switch power)
    // PC11: Output, Console Power Source Control(CON_P_BAT)電源選擇(Battery)
    // PC12: Output, Battery Connect Relay(BAT_REL)電池電源開關
    //
    // Analog Intput
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // PWM Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // Output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    //----------------------------------------------------------------------------
    GPIO_Reset() ;
    //----------------------------------------------------------------------------
    // PWM Initial
    /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 3 PWM signals with 3 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
    - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
    
    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
    = 24 MHz / 1200 = 20 KHz
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock/_PWM_FREQ_)-1 ;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0 ;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    //
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
    // 2014.02.25
    // ECB PWM
    /* -----------------------------------------------------------------------
    TIM2 Configuration: generate 1 PWM signals with 1 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
    - Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
    
    The TIM2 is running at 36 KHz: TIM2 Frequency = TIM2 counter clock/(ARR + 1)
    = 24 MHz / 1200 = 20 KHz
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock/_PWM_FREQ_)-1 ;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /* PWM1 Mode configuration: Channel 3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0 ;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
    //
    return ;
}



/*******************************************************************************
* Function Name  : GPIO_Reset
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Reset(void)
{
    //  
    if( LCBSystemProcessStatus.B.sErPAction == 0 )
    {
        ConsolePower12V(OFF) ;
        ExternalPower(OFF) ;
    }
    //
    ConsolePowerBattery(OFF) ;
    BatteryRelay(OFF) ;
    //   
    // All GPIO if OFF
    RS485Rx(RXD) ;
    VCC3V3(ON);
    //
    STATUSLED1(OFF) ;
    STATUSLED2(OFF) ;
    STATUSLED3(OFF) ;
    STATUSLED4(OFF) ;
    // 
    SK_EErom(LOW) ;
    DI_EErom(LOW) ;
    CS_EErom(LOW) ;
    // 
    BatteryForMCU(OFF) ;
    LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
    LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
    //
    return ; 
}



unsigned short TestPowerOnTime  ; // for Test Power on time
/*******************************************************************************
* Function Name  : LCBMain_SystemTick
* Description    : 1ms Interrupt
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_SystemTick(void)
{
    SystemTimeCounter += 1 ;
    CheckSaveEEPROMTime += 1 ; 
    UpdateLogFileTime += 1 ;
    // for Check Power On Time
    if( LCBSystemProcessStatus.B.sConsolePowerOn == 0 )
    {
        TestPowerOnTime += 1 ;
    }
    //----------------------------------------------------------------------------
    JHTLCBComm_TransmitData() ;
    if( LCBSystemProcessStatus.B.sErPAction == 1 )
    {// ERP mode
        return ;
    }
    ADC_Process() ;
    LCBMain_DCPluginCheck() ;
    RPM_CaptureTimeoutProcess() ;
    LCBMain_BoostController() ;
    SendTestPWMTime += 1 ;
    //----------------------------------------------------------------------------
    if( LCBSystemProcessStatus.B.sResetOK == 1 )
    {
        //------------------------------------------------------------------------
        LCBMain_ConsolePowerControl() ;
        
        if(_BoostHWcheck == 1)
        {// 昇壓迴路檢查完成後才執行
            LCBMain_BatteryChargeControl() ;
        }
        //------------------------------------------------------------------------
#ifdef  DebugMonitor
        //DebugADCOutTime += 1 ;
#endif
        
        //------------------------------------------------------------------------
    }
    else
    {
        //------------------------------------------------------------------------
        LCBMain_BurninCheck() ;
        //------------------------------------------------------------------------
    }
    //----------------------------------------------------------------------------
    // 電源指示燈顯示控制
    if( LCBSystemProcessStatus.B.sDCPlugin == 0 )
        StatusLED1Time = 1000 ;
    else
        StatusLED1Time = 2000 ;
    if( StatusLED1Time != 0 )
    {
        StatusLED1TimeCounter += 1 ;
        if( StatusLED1TimeCounter == (StatusLED1Time/2) )
        {
            STATUSLED1(ON) ;
        }
        if( StatusLED1TimeCounter >= StatusLED1Time )
        {
            STATUSLED1(OFF) ;
            StatusLED1TimeCounter = 0 ;
        }
    }
    else
    {
        StatusLED1TimeCounter = 0 ;
    }
    //
    // 2013.12.03
    
    // Check Max
    if( EepromControl.B.SaveLogMax == 0 )
    {
        if( LCBADC.GeneratorVoltage > 260 )
        {
            if( GeVMaxReportData.Para.Value < LCBADC.GeneratorVoltage  )
                GeVMaxReportData.Para.Value = LCBADC.GeneratorVoltage ;
            
            GeVMaxReportData.Para.AccTime += 1 ;
            EepromControl.B.UpdateLogMax = 1 ;
        }
        
        if( LCBADC.GeneratorCurrent > 370 )
        {
            if( GeCMaxReportData.Para.Value < LCBADC.GeneratorCurrent  )
                GeCMaxReportData.Para.Value = LCBADC.GeneratorCurrent ;
            
            GeCMaxReportData.Para.AccTime += 1 ;
            EepromControl.B.UpdateLogMax = 1 ;
        }
        
        if( LCBADC.BatteryChargeCurrnet > 50 )
        {
            if( BatCMaxReportData.Para.Value < LCBADC.GeneratorCurrent  )
                BatCMaxReportData.Para.Value = LCBADC.GeneratorCurrent ;
            
            BatCMaxReportData.Para.AccTime += 1 ;
            EepromControl.B.UpdateLogMax = 1 ;
        }
        
        if( LCBADC.ElectroMagnetCurrent > 200 )
        {
            if( EMCMaxReportData.Para.Value < LCBADC.ElectroMagnetCurrent  )
                EMCMaxReportData.Para.Value = LCBADC.ElectroMagnetCurrent ;
            
            EMCMaxReportData.Para.AccTime += 1 ;
            EepromControl.B.UpdateLogMax = 1 ;
        }
    }
    
    //
    
    // 電磁鐵控制
    EMControl_Running();
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
unsigned short LCBMain_CalculatorResistanceCommand(unsigned short EMCMD)
{
    unsigned short TempEMCommand = 0;
    
    if( EMCMD != 0 )
        TempEMCommand = (unsigned long)EMCMD * LCBADC.ResistanceOffset / 100 ;
    
    return TempEMCommand ;
}

/*******************************************************************************
* Function Name  : LCBMain_BurninCheck
* Description    : Check Burn In 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_BurninCheck(void)
{
    if( LCBSystemProcessStatus.B.sBurninCheck == 1 )
    {
        // Burn Test Hardware check
        if( iBurnin(0) )
        {
            BurnTestPinLoTime += 1 ;
            BurnTestPinHiTime = 0 ;
            if( BurnTestPinLoTime > 250 )
            {// 250ms
                LCBSystemProcessStatus.B.sBurninTest = 1 ;
                LCBSystemProcessStatus.B.sBurninCheckOK = 1 ;
                LCBSystemProcessStatus.B.sBurninCheck = 0 ;
            }
        }
        else
        {
            BurnTestPinLoTime = 0 ;
            BurnTestPinHiTime += 1 ;
            if( BurnTestPinHiTime > 250 )
            {// 250ms
                LCBSystemProcessStatus.B.sBurninTest = 0 ;
                LCBSystemProcessStatus.B.sBurninCheckOK = 1 ;
                LCBSystemProcessStatus.B.sBurninCheck = 0 ;
            }
        }
    }
    
    
    if( LCBSystemProcessStatus.B.sBurninTestRun == 1 )
        BurninTestTime += 1 ;
    else
        BurninTestTime = 0 ;
    
    return ;
}

/*******************************************************************************
* Function Name  : LCBMain_DCPluginCheck
* Description    : Check DC power plugin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_DCPluginCheck(void)
{ 
    if( iDCPlugin(DCPLUG) )
    {// 插市電
        DCPluginTestTime += 1 ;
        DCunPluginTestTime = 0 ;
        if( DCPluginTestTime > 250 )
        {//250ms
            DCPluginTestTime = 0 ;          
            ExternalPower(ON) ;// 打開外部電源
            MainPowerStableTimeLimit = _DCpowerOnStableTime ;
            
            if( LCBSystemProcessStatus.B.sDCPlugin == 0 && 
               LCBSystemProcessStatus.B.sConsolePowerOn == 1 )
            {//SP change to DC
                LCBSystemProcessStatus.B.sPowerSourceChangeToSP = 0 ;
                if( LCBSystemProcessStatus.B.sPowerSourceChangeToDC == 0 )
                    LCBSystemProcessStatus.B.sPowerSourceChangeToDC = 1 ;
            }
            if( LCBSystemProcessStatus.B.sCheckDCPlugin == 1 )
            {
                LCBStatus.bit.ACPluginStatus = 1 ;   
                LCBSystemProcessStatus.B.sCheckDCPluginOK = 1 ;
                LCBSystemProcessStatus.B.sCheckDCPlugin = 0 ;
                LCBSystemProcessStatus.B.sDCPlugin = 1 ;
            }
            //
        }
    }
    else
    {// 
        DCPluginTestTime = 0 ;
        DCunPluginTestTime += 1 ;
        if( DCunPluginTestTime > 250 )
        {
            DCunPluginTestTime = 0 ;
            
            ExternalPower(OFF) ;// 關閉外部電源
            MainPowerStableTimeLimit = _ACpowerOnStableTime ;
            if( LCBSystemProcessStatus.B.sDCPlugin == 1 && 
               LCBSystemProcessStatus.B.sConsolePowerOn == 1 )
            {//DC change to SP
                LCBSystemProcessStatus.B.sPowerSourceChangeToDC = 0 ;
                if( LCBSystemProcessStatus.B.sPowerSourceChangeToSP == 0 )
                    LCBSystemProcessStatus.B.sPowerSourceChangeToSP = 1 ;
            }
            if( LCBSystemProcessStatus.B.sCheckDCPlugin == 1 )
            {
                LCBStatus.bit.ACPluginStatus = 0 ;
                LCBSystemProcessStatus.B.sCheckDCPluginOK = 1 ;
                LCBSystemProcessStatus.B.sCheckDCPlugin = 0 ;
                LCBSystemProcessStatus.B.sDCPlugin = 0 ;
            }         
        }
    }
    return ;
}


/*******************************************************************************
* Function Name  : LCBMain_GetDCPluginStatus
* Description    : Return DC power plugin Status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_GetDCPluginStatus(void)
{
    if( LCBSystemProcessStatus.B.sDCPlugin == 1 )
        return 1 ;
    return 0 ;      
}


/*******************************************************************************
* Function Name  : LCBMain_GetDCPluginStatus
* Description    : Return DC power plugin Status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_GetConsolePowerStatus(void)
{
    if( LCBSystemProcessStatus.B.sConsolePowerOn == 1 )
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
void LCBMain_BoostController(void)
{
    //
    unsigned short Over ;
    unsigned short Under ;
    unsigned short TrackingTime ;
    
    //----------------------------------------------------------------------------
    if( BoostControllerStatus.B.Enable == 1 && LCBSystemProcessStatus.B.sErPAction == 0 )
    {
        // Quickly Power on check
        if( LCBSystemProcessStatus.B.sResetOK == 0 )
        {
            TrackingTime = 250 ;
        }
        else
        {
            TrackingTime = 500 ;
        }
        //
        switch(BoostControllerStatus.B.ControlMode)
        {
        case  BoostModeCV://Boost Hardware Check 使用
            if( BoostOutputVoltage > 1200 )
            {         
                BoostControllerTime += 1 ;
                if( BoostControllerTime >= TrackingTime ) // 500
                {
                    Over = 0 ;
                    Under = 0 ;
                    //
                    if( BoostPWMValue == 0 )
                        BoostPWMValue = 300 ;
                    else
                    {
                        //
                        if( LCBADC.BoostVoltage >= 1000 && LCBADC.BoostCurrent < _BoostMaxLimitedValue )
                        {
                            //
                            if( BoostOutputVoltage > LCBADC.BoostVoltage )
                            {
                                Under = BoostOutputVoltage - LCBADC.BoostVoltage ;
                            }
                            else
                            {
                                Over = LCBADC.BoostVoltage - BoostOutputVoltage  ;
                            }
                            //
                            if( Under > 5) //-0.05V
                            {
                                BoostControllerStatus.B.VoltageInRange = 0 ;
                                if( Under > 10 ) //-0.1V
                                    BoostPWMValue += Under ;
                                else
                                    BoostPWMValue += 2 ;
                            }
                            else
                            {
                                if( Over > 5 )//+0.05V
                                {
                                    BoostControllerStatus.B.VoltageInRange = 0 ;
                                    if( Over > 10 )//+0.1V
                                    {
                                        if( BoostPWMValue > Over )
                                            BoostPWMValue -= Over ;
                                        else
                                            BoostPWMValue = 0 ; 
                                    }
                                    else
                                    {
                                        if( BoostPWMValue > 2)
                                            BoostPWMValue -= 2 ;
                                        else
                                            BoostPWMValue = 0 ;  
                                    }
                                }
                                else
                                {
                                    BoostControllerStatus.B.VoltageInRange = 1 ;
                                }
                            }
                        }
                        else
                            BoostPWMValue = 0 ;
                    }
                    BoostControllerTime = 0 ;
                    if( (BoostPWMValue/10) > _BOOST_MAX_ )
                    {
                        BoostPWMValue = _BOOST_MAX_ * 10;
                        BoostPWM(_BOOST_MAX_) ;
                    }
                    else
                        BoostPWM((BoostPWMValue/10)) ;
                }
            }
            else
            {
                BoostControllerStatus.B.VoltageInRange = 0 ;
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
            }
            break ;
        case  BoostModeBCC:// Battery change control
            if( LCBSystemProcessStatus.B.sBatteryOK == 0 )
            {
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
                BatteryChargeOFF ;
                break ;
            }
            //
            if( BatteryChargeCurrentLimit != 0 )
            {
                BoostControllerTime += 1 ;
                if( BoostControllerTime >= 500 )
                {
                    if( BoostOutputVoltage > LCBADC.BoostVoltage && 
                       LCBADC.BoostCurrent < _BoostMaxLimitedValue )
                    {
                        //
                        Over = 0 ;
                        Under = 0 ;
                        //
                        if( LCBADC.BatteryChargeCurrnet < BatteryChargeCurrentLimit )
                        {
                            Under = BatteryChargeCurrentLimit - LCBADC.BatteryChargeCurrnet ;
                        }
                        else
                        {
                            Over = LCBADC.BatteryChargeCurrnet - BatteryChargeCurrentLimit  ;
                        }
                        //
                        if( Under >= 2) //-0.02A
                        {
                            if( Under >= 10 ) //-0.1V
                                BoostPWMValue += Under ;
                            else
                                BoostPWMValue += 2 ;
                        }
                        else
                        {
                            if( Over >= 2 )//+0.02A
                            {
                                if( Over >= 10 )//+0.1V
                                {
                                    if( BoostPWMValue > Over )
                                        BoostPWMValue -= Over ;
                                    else
                                        BoostPWMValue = 0 ; 
                                }
                                else
                                {
                                    if( BoostPWMValue > 2)
                                        BoostPWMValue -= 2 ;
                                    else
                                        BoostPWMValue = 0 ;  
                                }
                            }
                        }
                    }
                    else
                    {
                        if( BoostPWMValue > 2)
                            BoostPWMValue -= 2 ;
                        else
                            BoostPWMValue = 0 ; 
                    }
                    BoostControllerTime = 0 ;
                    if( BoostPWMValue > _BOOST_MAX_ )
                        BoostPWMValue = _BOOST_MAX_ ;
                    BoostPWM(BoostPWMValue) ;
                }
            }
            else
            {
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
            }
            break ;
        case  BoostModeLBCC:// 定電流
            //
            if( LCBSystemProcessStatus.B.sBatteryOK == 0 )
            {
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
                BatteryChargeOFF ;
                break ;
            }
            //
            if( BatteryChargeCurrentLimit != 0 )
            {
                BoostControllerTime += 1 ;
                if( BoostControllerTime >= 500 )
                {
                    BoostControllerTime = 0 ;
                    //
                    Over = 0 ;
                    Under = 0 ;
                    //
                    if( LCBADC.BatteryChargeCurrnet < BatteryChargeCurrentLimit )
                    {
                        Under = BatteryChargeCurrentLimit - LCBADC.BatteryChargeCurrnet ;
                    }
                    else
                    {
                        Over = LCBADC.BatteryChargeCurrnet - BatteryChargeCurrentLimit  ;
                    }
                    //
                    if( Under > 2) //-0.02A
                    {
                        if( Under > 10 ) //-0.1V
                            BoostPWMValue += Under ;
                        else
                            BoostPWMValue += 2 ;
                    }
                    else
                    {
                        if( Over > 2 )//+0.02A
                        {
                            if( Over > 10 )//+0.1V
                            {
                                if( BoostPWMValue > Over )
                                    BoostPWMValue -= Over ;
                                else
                                    BoostPWMValue = 0 ; 
                            }
                            else
                            {
                                if( BoostPWMValue > 2)
                                    BoostPWMValue -= 2 ;
                                else
                                    BoostPWMValue = 0 ;  
                            }
                        }
                    }
                }
                if( BoostPWMValue > _CHARGE_PWM_MAX_ )
                    BoostPWMValue = _CHARGE_PWM_MAX_ ;
                BatteryChargePWM(BoostPWMValue) ;
            }
            else
            {
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BatteryChargeOFF ;
            }
            break ;
        default:
            BoostControllerStatus.B.VoltageInRange = 0 ;
            BoostControllerTime = 0 ;
            BoostPWMValue = 0 ;
            BoostOFF ;
            break ;
        }          
    }
    else
    {
        BoostControllerStatus.B.Enable = 0 ;
        BoostControllerStatus.B.VoltageInRange = 0 ;
        BoostControllerTime = 0 ;
        BoostPWMValue = 0 ;
        BoostOFF ;
        BoostControlOCTime = 0 ;
    }
    //
    return ;
}




/*******************************************************************************
* Function Name  : LCBMain_BoostHardwareCheck()
* Description    : 昇壓電路檢測
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_BoostHardwareCheck(void)
{
    unsigned short StabeTime = 0 ;
    unsigned short TempTime = 0 ;
    unsigned short StabeTime1 = 0 ;
    unsigned short TempTime1 = 0 ;
    //
    if( ADC_GetCalculatorValue(_BoostVoltage15V_) >= 1000 )
    {
        BoostControllerStatus.B.ControlMode = BoostModeCV ;
        BoostControllerStatus.B.Enable = 1 ;
        BoostOutputVoltage = 1500 ;
        SystemTimeCounter = 0 ;
    }
    else
    {
        return BOOST_HW_ERR ;
    }
    //
    while(1)
    {
        if( BoostControllerStatus.B.VoltageInRange == 1 )
        {
            StabeTime1 = SystemTimeCounter - TempTime1 ;
            if( StabeTime1 > 200 ) // 500
            {
                BoostOutputVoltage = 0 ;
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerStatus.B.HardwareOK = 1 ;
                return BOOST_HW_OK ;
            }
        }    
        else
        {
            TempTime1 = SystemTimeCounter ;
            StabeTime1 = 0 ;
            StabeTime = SystemTimeCounter - TempTime ;
            if( StabeTime > 15000 )
            {
                BoostOutputVoltage = 0 ;
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerStatus.B.HardwareOK = 0 ;
                return BOOST_HW_ERR ;
            }
        }
        //
        RPM_CalculatorAnalogRPM() ;
#ifdef  DebugMonitor
        /*
        if( DebugADCOutTime > 500 )
        {
        FeedBackAdcData.RPM_Freq = RPMData.Frequency ;
        DebugADCOutTime = 0 ;
        DebugOut() ;
    }
        */
#else
        
        JHTLCBComm_RxProcess() ;
#endif
    }
}



/*******************************************************************************
* Function Name  : LCBMain_BatteryHardwareCheck
* Description    : 開機電池迴路檢查 (014A / 0248)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_BatteryHardwareCheck(void)
{
    //
    unsigned char Mode ;
    unsigned short TempTime ; 
    unsigned short TimeCounter ;
    //
    Mode = 0 ;
    while(1)
    {
        switch(Mode) 
        {
        default   :
            Mode = 1 ;
            TimeCounter = 0 ;
            SystemTimeCounter = 0 ;  
            TempTime = 0 ;
            BatteryRelay(OFF) ;// Close Battery charger relay
            LCBSystemProcessStatus.B.sBatteryConnect = 0 ;
        case 1    :  // Relay OFF 
            if( SystemTimeCounter > 500 ) // 0.5 Sec.
            {
                if( TimeCounter > 200 )
                {
                    // Add Error
                    ErrorCodeStatus.bit.EB014A = 1 ;
                    SystemTimeCounter = 0 ;
                    return BATTERY_HW_ERR ;
                }
                else
                {
                    TimeCounter = 0 ;
                    BatteryRelay(ON) ;
                    Mode = 2 ;
                    SystemTimeCounter = 0 ;  
                    TempTime = 0 ;
                }
            }
            else
            {
                if( SystemTimeCounter != TempTime )
                {
                    TempTime = SystemTimeCounter ;
                    if( ADC_GetCalculatorValue(_BatteryVoltage_) > 200 )
                        TimeCounter += 1 ;
                    else
                        TimeCounter = 0 ;
                }
            }
            break ;
        case 2    :// Check Relay ON
            if( ADC_GetDataSatndbyStatus() == 1 )
            {
                if( SystemTimeCounter > 500 )
                {
                    if( ADC_GetCalculatorValue(_BatteryVoltage_) < 600 )
                    {
                        BatteryRelay(OFF) ;
                        LCBSystemProcessStatus.B.sBatteryConnect = 0 ;
                        LCBSystemProcessStatus.B.sBatteryOK = 0 ;
                        // Add Error
                        ErrorCodeStatus.bit.EB0248 = 1 ;
                        SystemTimeCounter = 0 ;
                        return BATTERY_HW_ERR ;
                    }
                    else
                    {
                        LCBSystemProcessStatus.B.sBatteryConnect = 1 ;
                        LCBSystemProcessStatus.B.sBatteryOK = 1 ;
                        Mode = 3 ;
                        TempTime = 0 ;
                        SystemTimeCounter = 0 ;
                    }
                }
            }
            else
                SystemTimeCounter = 0 ;
            //
            break ;
        case 3    :
            return BATTERY_HW_OK ;
        }
        //
        RPM_CalculatorAnalogRPM() ;
#ifdef  DebugMonitor
        /*
        if( DebugADCOutTime > 500 )
        {
        FeedBackAdcData.RPM_Freq = RPMData.Frequency ;
        DebugADCOutTime = 0 ;
        DebugOut() ;
    }
        */
#else
        JHTLCBComm_RxProcess() ;
#endif
    }
    //
}

/*******************************************************************************
* Function Name  : LCBMain_ConsolePowerControl
* Description    : Console Power Control
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_ConsolePowerControl(void)
{ 
    unsigned long TempKeepTime ;
    static unsigned long selfPowerWaitTime = 0;
    
    if( RPMData.AverageRPM >= SP_CONSOLE_POWER_ON_RPM && LCBADC.SecondDC12 > 1130 )
    {
        if(selfPowerWaitTime < _ACpowerOnStableTime) ++selfPowerWaitTime;
    }
    else 
    {        
        selfPowerWaitTime = 0;
    }
    
    // Power source change process
    if( LCBSystemProcessStatus.B.sPowerSourceChangeToSP || LCBSystemProcessStatus.B.sPowerSourceChangeToDC)
    {
        LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
        LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
        LCBSystemProcessStatus.B.sWaitConsolePowerOn = 0 ;
        LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
        MainPowerStableTime = 0 ; 
        ConsolePowerBattery(OFF) ;
             
        if( LCBSystemProcessStatus.B.sPowerSourceChangeToSP && selfPowerWaitTime < _ACpowerOnStableTime)
        {
            ConsolePower12V(OFF); 
            LCBEeprom.Member.PowerOffTime = 1 ; 
            BatteryKeepTimeCounter = 0 ;
            BatteryKeepTime = 30000 ; 
            LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
            LCBSystemProcessStatus.B.sBatteryCharge = 0 ;           
            LCBSystemProcessStatus.B.sWaitConsolePowerOn = 1 ;     
            LCBSystemProcessStatus.B.sConsolePowerOn = 0 ;
            MainPowerStableTime = selfPowerWaitTime;         
            JHTLCBComm_ResetTimeoutStatus() ;
            JHTLCBComm_ClearAllErrorMessage(0);    
        }
        else if(LCBSystemProcessStatus.B.sPowerSourceChangeToDC)
        {
            ConsolePower12V(OFF); 
        }

        LCBSystemProcessStatus.B.sPowerSourceChangeToSP = 0 ;
        LCBSystemProcessStatus.B.sPowerSourceChangeToDC = 0 ;
        return;
    }
    
    // Wait power voltage stable
    if( LCBSystemProcessStatus.B.sWaitConsolePowerOn == 1 )
    {
        if( LCBSystemProcessStatus.B.sDCPlugin == 0 )  //Safe Power
        {
            if( LCBADC.SecondDC12 < 1130 )
            {
                MainPowerStableTime = 0 ;
            }
        }
        if( ++MainPowerStableTime > MainPowerStableTimeLimit ) //5000
        {
            LCBSystemProcessStatus.B.sWaitConsolePowerOn = 0 ;
            LCBSystemProcessStatus.B.sConsolePowerOn = 0 ;
            MainPowerStableTime = 0 ;
        }     
    }
    else
    {
        if( LCBSystemProcessStatus.B.sDCPlugin == 1 ) 
        {// DC Plugin process
            if( LCBSystemProcessStatus.B.sConsolePowerOn == 0 )
            {
                ExternalPower(ON) ;
                BatteryForMCU(OFF) ;
                LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
                LCBSystemProcessStatus.B.sConsolePowerOn = 1 ;
                LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower = 0 ;
                LCBSystemProcessStatus.B.sWaitBatteryOffHelpPower = 0 ;
                LCBSystemProcessStatus.B.sForceBatteryCharge = 1 ;  
                JHTLCBComm_ResetTimeoutStatus() ;
                JHTLCBComm_ClearAllErrorMessage(0); 
                ConsolePower12V(OFF) ;
            }
        }
        else
        { // self-power process
            if( LCBSystemProcessStatus.B.sConsolePowerOn == 0 )
            {
                // RPM Limit
                if( RPMData.AverageRPM >= SP_CONSOLE_POWER_ON_RPM )
                {
                    if( ++CheckRPMTime > 250 ) // 0.25Sec.
                    {
                        CheckRPMTime = 0 ;
                        ConsolePower12V(ON) ;
                        LCBSystemProcessStatus.B.sConsolePowerOn = 1 ;
                        LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower = 0 ;
                        LCBSystemProcessStatus.B.sWaitBatteryOffHelpPower = 0 ;
                        if( LCBSystemProcessStatus.B.sBatteryOK == 1 )
                        {
                            LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower = 1 ;
                            BatteryDelayTime = 0 ;
                        }
                        JHTLCBComm_ResetTimeoutStatus() ;
                        JHTLCBComm_ClearAllErrorMessage(0);    
                    }
                }
                else CheckRPMTime = 0 ;
            }
            else
            {
                if( LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower ==  1 )
                {
                    if( ++BatteryDelayTime > 500 ) //0.5 Sec.
                    {
                        BatteryDelayTime = 0 ;
                        if( LCBADC.BatteryVoltage > 1100 ) // 11V
                        {
                            LCBSystemProcessStatus.B.sWaitBatteryOffHelpPower = 1 ;
                            BatteryChargeOFF ;
                            BoostOFF ;
                            BoostControllerStatus.B.Enable = 0 ;
                            LCBSystemProcessStatus.B.sBatteryDischarge = 1 ;
                            LCBSystemProcessStatus.B.sForceBatteryCharge = 1 ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                            ConsolePowerBattery(ON) ;
                        }
                        LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower = 0 ;
                    }
                }
                else
                {
                    if( LCBSystemProcessStatus.B.sWaitBatteryOffHelpPower ==  1 )
                    {
                        if( ++BatteryDelayTime > 500 ) //0.5 Sec.
                        {
                            BatteryDelayTime = 0 ;
                            ConsolePowerBattery(OFF) ;
                            LCBSystemProcessStatus.B.sWaitBatteryOffHelpPower = 0 ;
                            LCBSystemProcessStatus.B.sWaitBatteryOnHelpPower = 0 ;
                            LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                        }
                    }
                    else
                    {
                        if( LCBEeprom.Member.PowerOffTime == 0 )
                        {
                            LCBEeprom.Member.PowerOffTime = 1 ;
                            BatteryKeepTimeCounter = 0 ;
                            BatteryKeepTime = 30000 ;
                            ConsolePowerBattery(OFF) ;
                            LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                            ConsolePower12V(OFF) ;
                            LCBSystemProcessStatus.B.sWaitConsolePowerOn = 1 ;
                            LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                            LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
                            BatteryForMCU(OFF) ;
                            LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
                            LCBSystemProcessStatus.B.sConsolePowerOn = 0 ;
                            MainPowerStableTime = 0 ; // 20130617
                            JHTLCBComm_ResetTimeoutStatus() ;
                        }
                        else
                        {
                            TempKeepTime = (unsigned long)LCBEeprom.Member.PowerOffTime * 60000 ;
                            if( TempKeepTime != BatteryKeepTime )
                            {
                                BatteryKeepTime = TempKeepTime ;
                            }
                            if( LCBSystemProcessStatus.B.sKeepTimeAction == 0 )
                            {
                                if( RPMData.AverageRPM < SP_BATTERY_POWER_RPM  || 
                                   ADC_GetCalculatorValue(_MainPower12V_) < SP_MAINPOWER_LIMIT || 
                                       LCBADC.GeneratorVoltage < SP_DCBUS_LIMIT )
                                {
                                    if( LCBSystemProcessStatus.B.sBatteryOK == 1 && LCBADC.BatteryVoltage > 1100 )
                                    {                                      
                                        ConsolePowerBattery(ON) ;
                                        BatteryForMCU(ON) ; 
                                        LCBSystemProcessStatus.B.sBatteryForMCU = 1 ;
                                        BatteryChargeOFF ;
                                        BoostOFF ;
                                        BoostControllerStatus.B.Enable = 0 ;
                                        LCBSystemProcessStatus.B.sBatteryDischarge = 1 ;
                                        LCBSystemProcessStatus.B.sForceBatteryCharge = 1 ;
                                        LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                        LCBSystemProcessStatus.B.sKeepTimeAction = 1 ;
                                        BatteryKeepTime = TempKeepTime ;
                                        BatteryKeepTimeCounter = 0 ;
                                        BatterySkipKeepTime = 0 ;
                                    }
                                }
                            }
                            else
                            {
                                //  
                                if( LCBSystemProcessStatus.B.sUpgradeMCU == 0 ) // Add 20140424
                                {
                                    if( RPMData.AverageRPM < SP_BATTERY_POWER_RPM )
                                        BatteryKeepTimeCounter += 1 ;
                                    else    
                                    {
                                        if( RPMData.AverageRPM >= SP_BATTERY_POWER_OFF_RPM && 
                                           LCBADC.GeneratorVoltage >= SP_DCBUS_LIMIT && 
                                               LCBADC.SecondDC12 >= SP_MAINPOWER_LIMIT )
                                        {
                                            BatterySkipKeepTime += 1 ;
                                            if( BatterySkipKeepTime > 2000 )
                                            {
                                                BatterySkipKeepTime = 0 ;
                                                BatteryKeepTimeCounter = 0 ;
                                                LCBSystemProcessStatus.B.sKeepTimeAction = 0 ;
                                                ConsolePowerBattery(OFF) ;
                                                //
                                                BatteryForMCU(OFF) ;
                                                LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
                                                //
                                                LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                                                LCBSystemProcessStatus.B.sBatteryCharge = 0 ;  
                                            }
                                        }
                                        else
                                            BatterySkipKeepTime = 0 ;
                                    }
                                    //  
                                    if( BatteryKeepTimeCounter > BatteryKeepTime || 
                                       LCBSystemProcessStatus.B.sBatteryLow == 1 || 
                                           JHTLCBComm_GetUCBOfflineStatus() == 1 || 
                                               JHTLCBComm_GetUCBOnlineStatus() == 0 )
                                    {
                                        BatteryKeepTimeCounter = 0 ;
                                        BatteryKeepTime = 30000 ;
                                        ConsolePowerBattery(OFF) ;
                                        LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                                        LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                        ConsolePower12V(OFF) ;
                                        LCBSystemProcessStatus.B.sWaitConsolePowerOn = 1 ;
                                        LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                                        LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
                                        //
                                        BatteryForMCU(OFF) ;
                                        LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
                                        LCBSystemProcessStatus.B.sKeepTimeAction = 0 ;
                                        LCBSystemProcessStatus.B.sConsolePowerOn = 0 ;
                                        //
                                        MainPowerStableTime = 0 ; // 20130617
                                        JHTLCBComm_ResetTimeoutStatus() ;
                                    }
                                }
                                else // Add 20140424
                                {
                                    BatteryKeepTimeCounter = 0 ;
                                    BatteryKeepTime = 30000 ;
                                    ConsolePowerBattery(OFF) ;
                                    LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                                    LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                    ConsolePower12V(OFF) ;
                                    LCBSystemProcessStatus.B.sWaitConsolePowerOn = 1 ;
                                    LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                                    LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
                                    //
                                    BatteryForMCU(OFF) ;
                                    LCBSystemProcessStatus.B.sBatteryForMCU = 0 ;
                                    LCBSystemProcessStatus.B.sKeepTimeAction = 0 ;
                                    LCBSystemProcessStatus.B.sConsolePowerOn = 0 ;
                                    //
                                    MainPowerStableTime = 0 ; // 20130617
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    //
    return ;
}


/*******************************************************************************
* Function Name  : LCBMain_BatteryChargeControl
* Description    : Battery Charge Control
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_BatteryChargeControl(void)
{
    //----------------------------------------------------------------------------
    // 20130327
    if( ErrorCodeStatus.bit.EB014A == 1 || ErrorCodeStatus.bit.EB0248 == 1 )
    {
        LCBSystemProcessStatus.B.sBatteryChargeEnable = 0 ;
        BoostOFF ;
        BatteryChargeOFF ;          
        BoostOutputVoltage = 0 ;
        BoostControllerStatus.B.Enable = 0 ;
        BoostControllerStatus.B.ControlMode = BoostModeBCC ;
        LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
        BatteryOCTimeCounter = 0 ;
        BatteryChargeHVTC = 0 ;
        BatteryChargeCheckTime = 0 ;
        BatteryChargeCurrentLimit = 0 ;
        BatteryFloatChargeCheckTime = 0 ;
        return ;
    }
    
    if( LCBSystemProcessStatus.B.sDCPlugin == 0 )
    {// safe power
        if( RPMData.AverageRPM < LCBEeprom.Member.LimitRpmForCharge )
        {
            LCBSystemProcessStatus.B.sBatteryChargeEnable = 0  ;
        }
        else
        {
            if( RPMData.AverageRPM >= (LCBEeprom.Member.LimitRpmForCharge+5) )
                LCBSystemProcessStatus.B.sBatteryChargeEnable = 1 ;
        }
    }
    else
    { 
        LCBSystemProcessStatus.B.sBatteryChargeEnable = 1 ;  //if(LCBADC.SecondDC12 < 1150)  sBatteryChargeEnable = 0   
    }
    //
    if( LCBSystemProcessStatus.B.sConsolePowerOn == 1 && 
       LCBSystemProcessStatus.B.sErPAction == 0 && 
           LCBSystemProcessStatus.B.sBatteryChargeEnable == 1 ) 
    {
        // Check Battery Voltage (Low Battery Status)
        if( LCBADC.BatteryVoltage < 1100 )
        {
            BatteryLowVoltageOffTC = 0 ;
            BatteryLowVoltageOnTC += 1 ;
            if( BatteryLowVoltageOnTC > 2000 )
            {
                BatteryLowVoltageOnTC = 0 ;
                LCBStatus.bit.BatteryLowStatus = 1 ;
                LCBSystemProcessStatus.B.sBatteryLow = 1 ;
            }
        }
        else
        {
            BatteryLowVoltageOnTC = 0 ;
            BatteryLowVoltageOffTC += 1 ;
            if( BatteryLowVoltageOffTC > 2000 )
            {
                BatteryLowVoltageOffTC = 0 ;
                LCBStatus.bit.BatteryLowStatus = 0 ;
                LCBSystemProcessStatus.B.sBatteryLow = 0 ;
            } 
        }
        //------------------------------------------------------------------------
        // Battery Charge
        if( LCBSystemProcessStatus.B.sBatteryOK == 1 && 
           BoostControllerStatus.B.HardwareOK == 1 && 
               BoostControllerStatus.B.OC == 0 )
        {
            if( LCBSystemProcessStatus.B.sBatteryDischarge == 0 && 
               LCBSystemProcessStatus.B.sBatteryCharge == 0 )
            {
                if( LCBADC.BatteryVoltage <= 1250 || 
                   LCBSystemProcessStatus.B.sForceBatteryCharge == 1 )
                {
                    BatteryFloatChargeCheckTime = 0 ;
                    BatteryChargeCheckTime += 1 ;
                    if( BatteryChargeCheckTime > 5000 )
                    {
                        LCBSystemProcessStatus.B.sForceBatteryCharge = 0 ;
                        BatteryChargeCheckTime = 0 ;
                        LCBSystemProcessStatus.B.sBatteryCharge = 1 ;
                        LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                        BatteryChargeCurrentLimit = 0 ;
                        BatteryChargeHVTC = 0 ;
                    }
                }
                else
                {
                    BatteryChargeCheckTime = 0 ;
                    if( LCBADC.BatteryVoltage > 1300 &&  
                       LCBADC.BatteryVoltage < 1340 ) // add by 20121129 LCBADC.BatteryVoltage < 1340
                    {
                        BatteryFloatChargeCheckTime += 1 ;
                        if( BatteryFloatChargeCheckTime > 5000 )
                        {
                            BatteryFloatChargeCheckTime = 0 ;
                            LCBSystemProcessStatus.B.sForceBatteryCharge = 0 ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 1 ;
                            LCBSystemProcessStatus.B.sBatteryFloatCharge = 1 ;
                            BatteryChargeCurrentLimit = 0 ;
                            BatteryChargeHVTC = 0 ;
                        }
                    }
                    else
                    {
                        if( LCBSystemProcessStatus.B.sDCPlugin == 1 )
                        {// DC Plugin process
                            LCBSystemProcessStatus.B.sForceBatteryCharge = 0 ;
                            BatteryChargeCheckTime = 0 ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 1 ;
                            LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                            BatteryChargeCurrentLimit = 0 ;
                            BatteryChargeHVTC = 0 ;
                        }
                    }
                }
            }
            else
            {
                // Charge Control
                if( LCBSystemProcessStatus.B.sBatteryCharge == 1 )
                {
                    //
                    if( LCBSystemProcessStatus.B.sBatteryFloatCharge == 1 )
                    {// 定電流迴路
                        if( BatteryChargeCurrentLimit == 0 )
                        {
                            if( LCBADC.BatteryVoltage < 1350 )
                            {
                                BatteryChargeHVTC += 1 ;
                                BatteryChargeFLTC = 0 ;
                                if( BatteryChargeHVTC > 5000 )
                                {
                                    BatteryChargeHVTC = 0 ;
                                    BatteryChargeCurrentLimit = 20 ; // 0.2 mA
                                    BoostOutputVoltage = 1400 ;
                                    BoostControllerStatus.B.Enable = 1 ;
                                    BatteryChargeON ;
                                }
                            }
                            else
                            {
                                BatteryChargeHVTC = 0 ;
                                BatteryChargeFLTC += 1 ;
                                if( BatteryChargeFLTC > 60000 )
                                {
                                    BatteryChargeFLTC = 0 ;
                                    BoostOutputVoltage = 0 ;                                      
                                    BoostOFF ;                              
                                    BoostControllerStatus.B.Enable = 0 ;
                                    BatteryChargeOFF ;
                                    BatteryChargeCurrentLimit = 0 ; 
                                    LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                    LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ; 
                                }
                            }
                        }
                        else
                        {
                            // Check Over Charge 
                            if( LCBADC.BatteryVoltage >= 1440 )
                            {
                                //   
                                BatteryChargeFLTC = 0 ;
                                BatteryChargeHVTC += 1 ;
                                if( BatteryChargeHVTC > 500 )
                                {
                                    BatteryChargeHVTC = 0 ;
                                    BoostOutputVoltage = 0 ;                                      
                                    BoostOFF ;                              
                                    BoostControllerStatus.B.Enable = 0 ;
                                    BatteryChargeOFF ;
                                    BatteryChargeCurrentLimit = 0 ; 
                                    LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                    LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ; 
                                    //
                                }
                            }
                            else
                            {
                                BatteryChargeHVTC = 0 ;
                                if( LCBADC.BatteryVoltage >= 1350 )
                                {
                                    BatteryChargeFLTC += 1 ;
                                    if( BatteryChargeFLTC > 60000 )
                                    {
                                        BatteryChargeFLTC = 0 ;
                                        BoostOutputVoltage = 0 ;                                      
                                        BoostOFF ;                              
                                        BoostControllerStatus.B.Enable = 0 ;
                                        BatteryChargeOFF ;
                                        BatteryChargeCurrentLimit = 0 ; 
                                        LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                        LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                                    }
                                }
                                else
                                    BatteryChargeFLTC = 0 ;
                            }
                        }
                        //
                    }
                    else
                    {// 定電壓迴路
                        if( LCBSystemProcessStatus.B.sDCPlugin == 0 )
                        {// Self-powered
                            
                            if( BatteryChargeCurrentLimit == 0 )
                            {
                                if( LCBADC.BatteryVoltage > 1130 ) // 1190
                                {
                                    BoostOFF ; 
                                    BatteryChargeOFF ;
                                    BoostOutputVoltage = 1470 ;
                                    BatteryChargeCurrentLimit = 40 ; // 0.40 mA
                                    BoostControllerStatus.B.ControlMode = BoostModeBCC ;
                                    BoostControllerStatus.B.Enable = 1 ;
                                    BatteryChargeON ;
                                }
                                else
                                {
                                    BoostOutputVoltage = 1380 ;
                                    BatteryChargeCurrentLimit = 20 ;// 0.2 mA
                                    BoostControllerStatus.B.ControlMode = BoostModeLBCC ; 
                                    BoostControllerStatus.B.Enable = 1 ;
                                    BatteryLVChargeTime = 0 ;
                                }
                            }
                            else
                            { // 20121217 Add check Voltage over 11V
                                if( BoostControllerStatus.B.ControlMode == BoostModeLBCC )
                                {
                                    if( LCBADC.BatteryVoltage >= 1160 )// 1200
                                    {
                                        BatteryLVChargeTime += 1 ;
                                        if( BatteryLVChargeTime > 30000 )
                                        {
                                            BoostControllerStatus.B.Enable = 0 ;
                                            BatteryLVChargeTime = 0 ;
                                            BatteryChargeOFF ;
                                            BoostOFF ;                                           
                                            BatteryChargeCurrentLimit = 0 ;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            // DC Plug-in
                            if( BatteryChargeCurrentLimit == 0 )
                            {
                                BatteryChargeCurrentLimit = 40 ;  //0.40
                                BoostControllerStatus.B.ControlMode = BoostModeBCC ;
                                BoostControllerStatus.B.Enable = 1 ;
                                BoostOutputVoltage = 1500 ;
                                BatteryChargeON ;
                            }
                        }
                        // Check Over 14.40V
                        if( LCBADC.BatteryVoltage >= 1440 )
                        {
                            //   
                            BatteryChargeHVTC += 1 ;
                            if( BatteryChargeHVTC > 500 )
                            {
                                BatteryChargeHVTC = 0 ;
                                BoostOutputVoltage = 0 ;                                      
                                BoostOFF ;                              
                                BoostControllerStatus.B.Enable = 0 ;
                                BatteryChargeOFF ;
                                //                              
                                LCBSystemProcessStatus.B.sBatteryFloatCharge = 1 ;
                                BatteryChargeCurrentLimit = 0 ; 
                                //
                            }
                        }
                        else
                        {
                            BatteryChargeHVTC = 0 ;
                        }
                        //
                    }
                    // OC Check
                    if( LCBADC.BatteryChargeCurrnet > 100 )// 1A
                    {
                        BatteryLCTimeCounter = 0 ;
                        BatteryOCTimeCounter += 1 ;
                        if( BatteryOCTimeCounter > 500 )
                        {
                            BatteryOCTimeCounter = 0 ;
                            BatteryChargeHVTC = 0 ;
                            BoostOutputVoltage = 0 ;                                      
                            BoostOFF ;                              
                            BoostControllerStatus.B.Enable = 0 ;
                            BatteryChargeOFF ;
                            LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                            LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ; 
                            BatteryChargeCurrentLimit = 0 ;
                            // Add Error
                            ErrorCodeStatus.bit.EB014A = 1 ;
                            //
                            BatteryRelay(OFF) ; // 20130327
                        }
                    }
                    else
                    {
                        BatteryOCTimeCounter = 0 ; 
                        // less charge check 20130327
                        if( BoostControllerStatus.B.Enable == 1 && BoostIO != 0 && BatteryChargeIO != 0 )
                        {
                            if( LCBADC.BatteryChargeCurrnet < 2 )
                            {
                                BatteryLCTimeCounter += 1 ;
                                if( BatteryLCTimeCounter > 5000 )
                                {
                                    BatteryOCTimeCounter = 0 ;
                                    BatteryChargeHVTC = 0 ;
                                    BoostOutputVoltage = 0 ;                                      
                                    BoostOFF ;                              
                                    BoostControllerStatus.B.Enable = 0 ;
                                    BatteryChargeOFF ;
                                    LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                                    LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ; 
                                    BatteryChargeCurrentLimit = 0 ;
                                }
                            }
                            else
                                BatteryLCTimeCounter = 0 ;
                        }
                        else
                            BatteryLCTimeCounter = 0 ;
                    }
                    //
                }
                else
                {
                    BoostOFF ;
                    BatteryChargeOFF ;
                    BoostOutputVoltage = 0 ;
                    BoostControllerStatus.B.Enable = 0 ;
                    BoostControllerStatus.B.ControlMode = BoostModeBCC ;
                    LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                    BatteryOCTimeCounter = 0 ;
                    BatteryChargeHVTC = 0 ;
                    BatteryChargeCheckTime = 0 ;
                    BatteryChargeCurrentLimit = 0 ;
                    BatteryFloatChargeCheckTime = 0 ;
                }
            }
        }
        else
        {
            BoostOFF ;
            BatteryChargeOFF ;          
            BoostOutputVoltage = 0 ;
            BoostControllerStatus.B.Enable = 0 ;
            BoostControllerStatus.B.ControlMode = BoostModeBCC ;
            LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
            BatteryOCTimeCounter = 0 ;
            BatteryChargeHVTC = 0 ;
            BatteryChargeCheckTime = 0 ;
            BatteryChargeCurrentLimit = 0 ;
            BatteryFloatChargeCheckTime = 0 ;
        }
    }
    else
    {
        //
        BoostOFF ;
        BatteryChargeOFF ;
        BoostOutputVoltage = 0 ;
        BoostControllerStatus.B.Enable = 0 ;      
        LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
        LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
        LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
        BatteryOCTimeCounter = 0 ;
        BatteryChargeHVTC = 0 ;
        BatteryChargeCheckTime = 0 ;
        BatteryChargeCurrentLimit = 0 ;
        BatteryFloatChargeCheckTime = 0 ;
        //
    }
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
void LCBMain_SetCutOffResistance(void)
{
    if( LCBSystemProcessStatus.B.sCutOffResistance != 1 )
        LCBSystemProcessStatus.B.sCutOffResistance = 1 ;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_GetBatteryCapacity(void)
{
    // Set Close Voltage
    //11.60 ~ 11.75V	10%
    //11.75 ~ 11.90V	20%
    //11.90 ~ 12.05V	30%
    //12.05 ~ 12.20V	40%
    //12.20 ~ 12.50V	50%
    //12.35 ~ 12.50V	60%
    //12.50 ~ 12.65V	70%
    //12.65 ~ 12.80V	80%
    //12.80 ~ 13.00V	90%
    //>13V	100%
    unsigned char Capacity = 0 ;
    if( LCBADC.BatteryVoltage > 1300 )		// 13.00V
        Capacity = 100 ;
    else
    {
        if( LCBADC.BatteryVoltage >= 1280 )	// > 12.80V
            Capacity = 90 ;
        else
        {
            if( LCBADC.BatteryVoltage >= 1265 )	// 12.65V
                Capacity = 80 ;
            else
            {
                if( LCBADC.BatteryVoltage >= 1250 ) 
                    Capacity = 70 ;
                else
                {
                    if( LCBADC.BatteryVoltage >= 1235 ) 
                        Capacity = 60 ;
                    else
                    {
                        if( LCBADC.BatteryVoltage >= 1220 ) 
                            Capacity = 50 ;
                        else
                        {
                            if( LCBADC.BatteryVoltage >= 1205 ) 
                                Capacity = 40 ;
                            else
                            {
                                if( LCBADC.BatteryVoltage >= 1190 ) 
                                    Capacity = 30 ;
                                else
                                {
                                    if( LCBADC.BatteryVoltage >= 1175 ) 
                                        Capacity = 20 ;
                                    else
                                    {
                                        if( LCBADC.BatteryVoltage >= 1160 ) 
                                            Capacity = 10 ;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }    
    }
    return Capacity ;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_SetErPStatus(unsigned char SetStatus)
{
    if( SetStatus != 0 )
        LCBSystemProcessStatus.B.sErPAction = 1 ;
    else
        LCBSystemProcessStatus.B.sErPAction = 0 ;
    return ;
}


/*******************************************************************************
* Function Name  : 
* Description    :
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char LCBMain_GetErPStatus(void)
{
    if(LCBSystemProcessStatus.B.sErPAction == 1 )
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
unsigned char LCBMain_GetResetOKStatus(void)
{
    if(LCBSystemProcessStatus.B.sResetOK == 1 )
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
void LCBMain_SetUpgradeStatus(void)
{
    if( LCBSystemProcessStatus.B.sUpgradeMCU == 0 )
        LCBSystemProcessStatus.B.sUpgradeMCU = 1 ;
    
    return ;
}

/*******************************************************************************
* Function Name  : lcb_main
* Description    : LCB Main Loop
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void lcb_main(void)
{
    //unsigned short TempPWM = 0xff ;
    unsigned short FromUCBWakeupTime = 0 ;
    unsigned short TmepTime = 0 ;
    unsigned char  HiLoStatus = 0 ;
    static unsigned long  HiLoTime = 0 ;
    static unsigned char  HiLoOkCount = 0 ;
    unsigned short TempOffset ;
    
    EXTI_InitTypeDef   EXTI_InitStructure;
    
    System_Initial();
    
    while(1)
    {
        switch( LCBMainMode )
        {
        case  MRESET:
            // Check DC Plugin
            if( LCBSystemProcessStatus.B.sCheckDCPluginOK == 1 ) 
            {
                // Check BurnIn
                if( LCBSystemProcessStatus.B.sBurninCheckOK == 1 ) 
                {
                    if( LCBSystemProcessStatus.B.sDCPlugin == 1 )
                        LCBMainMode = DCPOWER_ON ;
                    else
                    {
                        LCBMainMode = SELFPOWER_ON ;
                        LCBSubMode = CHECK_MAIN_POWER ;
                        SystemTimeCounter = 0 ;
                    }
                }
                else
                {
                    if( LCBSystemProcessStatus.B.sBurninCheck == 0 )
                        LCBSystemProcessStatus.B.sBurninCheck = 1 ;
                }
            }
            else
            {
                if( LCBSystemProcessStatus.B.sCheckDCPlugin == 0 )
                    LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
            }
            break ;
        case  DCPOWER_ON:// 插電系統
            LCBMainMode = SELFPOWER_ON ;
            LCBSubMode = CHECK_MAIN_POWER ;
            SystemTimeCounter = 0 ;
            break ;                                
        case  SELFPOWER_ON:
            if( LCBSystemProcessStatus.B.sBurninTest == 0 )
            {
                // Test Boost Hardware
                switch(LCBSubMode)
                {
                default:
                    LCBSubMode = CHECK_MAIN_POWER ;
                    SystemTimeCounter = 0 ;
                case CHECK_MAIN_POWER:// 12V電源測試 
                    if( LCBSystemProcessStatus.B.sDCPlugin == 0 )
                    {   // safe power
                        // Wait Main Power Stable to 11.3V at the 5S
                        if( LCBADC.SecondDC12 > 1130 ) 
                        {
                            if( SystemTimeCounter > _ACpowerOnStableTime )
                            {
                                SystemTimeCounter = 0 ;
                                if( RPMData.AverageRPM >= SP_CONSOLE_POWER_ON_RPM )
                                {
                                    LCBSubMode = CHECK_BATTERY_HW ;
                                    ConsolePower12V(ON) ;
                                }
                            }
                        }
                        else
                            SystemTimeCounter = 0 ;
                    }
                    else // skip check dc voltage
                    {// DC plug in
                        if( SystemTimeCounter > _DCpowerOnStableTime )
                        {
                            LCBSubMode = CHECK_BATTERY_HW ;
                        }
                    }
                    break ;
                case CHECK_BATTERY_HW:// 電池迴路檢查
                    LCBMain_BatteryHardwareCheck() ;
                    if( LCBSystemProcessStatus.B.sDCPlugin == 1 )
                    {
                        LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                        LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
                        LCBSubMode = CHECK_DCPLUGIN ;
                    }
                    else
                        LCBMainMode = MAIN_POWER_OK ;
                    break ;
                case CHECK_DCPLUGIN:
                    if( LCBSystemProcessStatus.B.sCheckDCPluginOK == 1 ) 
                    {
                        LCBMainMode = MAIN_POWER_OK ;
                    }
                    else
                    {
                        if( LCBSystemProcessStatus.B.sCheckDCPlugin == 0 )
                            LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                    }
                    break ;
                }
                //
            }
            else
                LCBMainMode = BURNIN_TEST ; 
            break ;
        case  MAIN_POWER_OK:
            LCBSystemProcessStatus.B.sResetOK = 1 ;
            LCBMainMode = CONSOLE_POWER ;
            break ;
        case  CONSOLE_POWER:    
            if( LCBSystemProcessStatus.B.sWaitConsolePowerOn == 0 )
            {
                MainPowerStableTime = MainPowerStableTimeLimit ;
                LCBSystemProcessStatus.B.sWaitConsolePowerOn = 1 ;
            }
            LCBMainMode = NORMAL ;
            SystemTimeCounter = 0 ;
            break ;
        case  BURNIN_TEST:
            if( LCBSystemProcessStatus.B.sBurninTest == 1 )
            {
                LCBSystemProcessStatus.B.sBurninTestRun = 1 ;
                if( RPMData.AverageRPM >= 60 )
                {            
                    if( BurninTestTime < 300000 )//900000
                    {
                        BurninTestEM = 180 ; // 220 ( old board )
                    }
                    else
                    {
                        BurninTestEM = 0 ;
                        if( BurninTestTime > 600000 )//1800000
                            BurninTestTime = 0 ;
                    }
                    if( SendData.EM_CurrentCommand != BurninTestEM )
                        SendData.EM_CurrentCommand = BurninTestEM ;
                }
                else
                {
                    SendData.EM_CurrentCommand = 0 ;
                    BurninTestTime = 0 ;
                }
            }
            else
            {
                // 
                if( LCBSystemProcessStatus.B.sBurninTestRun == 1 )
                {
                    LCBSystemProcessStatus.B.sBurninTestRun = 0 ;
                    BurninTestEM = 0 ;
                    SendData.EM_CurrentCommand = 0 ;
                    BurninTestTime = 0 ;
                }
                //
            }
            if( SendTestPWMTime > 250 )
            {
                SendTestPWMTime = 0 ;  
                // 傳送控制阻力值  
                EcbPWM(SendData.EM_CurrentCommand);    
                //
            }
            break ;
        case  NORMAL:
            break ;
        case  POWER_SAVE:        
            //------------------------------------------------------------------------
            switch( LCBSubMode )
            {
            default:
                LCBSubMode = SLEEP_MODE ;
                SystemTimeCounter = 0 ;
            case SLEEP_MODE:
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerStatus.B.VoltageInRange = 0 ;
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
                LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                BatteryOCTimeCounter = 0 ;
                BatteryChargeHVTC = 0 ;
                BatteryChargeCheckTime = 0 ;
                BatteryChargeCurrentLimit = 0 ;
                BatteryFloatChargeCheckTime = 0 ;
                LCBSystemProcessStatus.B.sResetOK = 0 ;
                BatteryChargeOFF ;
                while( SystemTimeCounter < 1000 ) ;
            case SLEEP_MODE1:    
                ExternalPower(ON);
                VCC3V3(OFF);
                
                ConsolePower12V(OFF);
                BatteryForMCU(OFF);
                EcbOFF;
                
                ConsolePowerBattery(OFF) ;
                BatteryRelay(OFF) ;
                RS485Rx(RXD) ;
                STATUSLED1(OFF) ;
                STATUSLED2(OFF) ;
                STATUSLED3(OFF) ;
                STATUSLED4(OFF) ;
                SK_EErom(HIGH) ;
                DI_EErom(HIGH) ;
                CS_EErom(HIGH) ;
                //
                ///*
                BoostControllerStatus.B.Enable = 0 ;
                BoostControllerStatus.B.VoltageInRange = 0 ;
                BoostControllerTime = 0 ;
                BoostPWMValue = 0 ;
                BoostOFF ;
                //
                //
                LCBSystemProcessStatus.B.sBatteryDischarge = 0 ;
                LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                LCBSystemProcessStatus.B.sBatteryFloatCharge = 0 ;
                BatteryOCTimeCounter = 0 ;
                BatteryChargeHVTC = 0 ;
                BatteryChargeCheckTime = 0 ;
                BatteryChargeCurrentLimit = 0 ;
                BatteryFloatChargeCheckTime = 0 ;
                LCBSystemProcessStatus.B.sResetOK = 0 ;
                BatteryChargeOFF ;
                //
                JHTLCBComm_ClearAllErrorMessage(1) ; // 20121003
                //
                EXTI_DeInit() ;
                PWR_ClearFlag(PWR_FLAG_WU);
                PWR_ClearFlag(PWR_FLAG_SB);
                //
                //====>> 為配合MCU省電模式設定使用,作為外部訊號觸發喚醒用
                // 0: PA
                // 1: PB
                // 2: PC
                // 3: PD
                // 4: PE
                // 5: PF
                // 6: PG
                //                                   
                // AFIO->EXTICR[0] 0~3      >> 0x0000xxxx
                // AFIO->EXTICR[1] 4~7      >> 0x0000xxxx
                // AFIO->EXTICR[2] 8~11     >> 0x0000xxxx
                // AFIO->EXTICR[3] 12~15    >> 0x0000xxxx
                AFIO->EXTICR[0] = 0x00000000 ;  // PA0 -- RPM
                // PA3 -- RXD
                // 
                /* Configure Key Button EXTI Line to generate an interrupt on falling edge */  
                EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line3 ;
                EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
                EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
                EXTI_InitStructure.EXTI_LineCmd = ENABLE ;
                EXTI_Init(&EXTI_InitStructure);
                
                /* Request to enter STOP mode with regulator in low power mode*/
                PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);
                EXTI_DeInit();
                // WAKEUP CHECK
                SystemInit() ;	
                LCBMainMode = POWER_SAVE ; 
                LCBSubMode = WAKEUP_CHECK ;
                
                //-------------------------------------------------------------------------                                      
                break ;
            case  WAKEUP_CHECK:    
                // 
                SystemTimeCounter = 0 ;
                FromUCBWakeupTime = 0 ;
                TmepTime = 0 ;
                HiLoTime = 0 ;
                HiLoStatus = 0 ;
                LCBMainMode = POWER_SAVE ; 
                LCBSubMode = SLEEP_MODE1 ;
                //
                while( SystemTimeCounter <= _WakeupCheckTime )
                {
                    // Check Wakeup from UCB
                    if( iWakeupFucb(0) )
                    {// RX signal
                        FromUCBWakeupTime += SystemTimeCounter - TmepTime ;
                        TmepTime = SystemTimeCounter ;
                        if( FromUCBWakeupTime > 50 )
                        {
                            FromUCBWakeupTime = 0 ;
                            LCBMainMode = POWER_SAVE ; 
                            LCBSubMode = WAKEUP_LCB ;
                            break ;
                        }
                    }
                    else
                    {
                        FromUCBWakeupTime = 0 ;
                        TmepTime = SystemTimeCounter ;
                    }
                    // Check Wakeup from RPM                                 
                    if( HiLoStatus == 0 )
                    {
                        if( iWakeupFRPM(0) )
                        {
                            HiLoTime += 1 ;
                        }
                        else  
                        {
                            HiLoStatus = 1 ;
                        }
                    }
                    else
                    {
                        if( iWakeupFRPM(1) )
                        {
                            HiLoTime += 1 ;
                        }
                        else
                        {
                            // 26~140 RPM
                            if( HiLoTime < 50500 && HiLoTime > 5000 ) //9500
                            {
                                HiLoOkCount += 1 ;
                                if( HiLoOkCount > 50 )
                                {
                                    LCBMainMode = POWER_SAVE ; 
                                    LCBSubMode = WAKEUP_UCB ;
                                    break ;
                                }
                            }
                            HiLoTime = 0 ;
                            HiLoStatus = 0 ;
                        }
                    }
                }
                break ;
            case  WAKEUP_UCB:                                                          
            case  WAKEUP_LCB:
                GPIO_Configuration() ;
                EMControl_Initial();
                RPM_Initial() ;
                //----------------------------------------------------------------------------
                // ADC
                ADC_Initial() ;
                //
                // UART
                JHTLCBComm_Initial() ;
                JHTLCBComm_HW_Initial() ;
                // Setup SysTick Timer for 1 msec interrupts 
                SysTick_Config(SystemCoreClock / 1000);
                // NVIC Configuration 
                NVIC_Configuration();  
                //GPIO_Reset() ;
                //
                if( LCBSubMode == WAKEUP_LCB )
                {
                    LCBMainMode = POWER_SAVE ; 
                    LCBSubMode = WAIT_UCB_WAKEUP ;
                    SystemTimeCounter = 0 ;
                }
                else
                {
                    // issue not wakeup from RPM
                    ConsolePower12V(OFF) ;
                    ExternalPower(OFF) ;
                    MainPowerStableTime = 0 ; // 20130617
                    SystemTimeCounter = 0 ;
                    LCBMainMode = POWER_SAVE ; 
                    LCBSubMode = WAKEUP_FRPM ;
                }
                break ;
            case  WAKEUP_FRPM:
                if( SystemTimeCounter > _LCBwakeupUCBstableTime )
                {
                    LCBSystemProcessStatus.All = 0 ;
                    LCBMainMode = MRESET ; //NORMAL ; 
                }
                break ;
            case  WAIT_UCB_WAKEUP:
                if( SystemTimeCounter < _UCBwakeupLCBwaitTime )
                {
                    //      
#ifdef  DebugMonitor
                    /*
                    if( DebugADCOutTime > 500 )
                    {
                    FeedBackAdcData.RPM_Freq = RPMData.Frequency ;
                    DebugADCOutTime = 0 ;
                    DebugOut() ;
                }
                    */
#else
                    JHTLCBComm_RxProcess() ;
#endif
                    if( LCBSystemProcessStatus.B.sErPAction == 0 )
                    {
                        LCBSystemProcessStatus.All = 0 ;
                        LCBMainMode = MRESET ; //NORMAL ;
                    }
                    //
                }
                else
                {
                    SystemTimeCounter = 0 ;
                    LCBMainMode = POWER_SAVE ; 
                    LCBSubMode = SLEEP_MODE1 ;
                } 
                break ;                                                       
            }
            break ;          
        default:
            LCBMainMode = MRESET ;
            break ;
        }
        if( LCBSystemProcessStatus.B.sErPAction == 0 )
        {// 非ERP
            //  RPM換算
            RPM_CalculatorAnalogRPM() ;
            //      
#ifdef  DebugMonitor
            /*
            if( DebugADCOutTime > 500 )
            {
            FeedBackAdcData.RPM_Freq = RPMData.Frequency ;
            DebugADCOutTime = 0 ;
            DebugOut() ;
        }
            */
#else
            // 數位通訊 
            JHTLCBComm_RxProcess() ;
#endif
            // Check Em Status
            if( EepromControl.B.SaveErrorLog == 0 && FeedBackAdcData.Status.B.EM_OverCurrent == 0 )
            {
                ErrorReportData.Para.EM_Current = ADC_GetCalculatorValue(_ECBCurrent_) ;//FeedBackAdcData.ElectroMagnetCurrent
            }
            if( FeedBackAdcData.Status.B.EM_OverCurrent == 1 )
            {
                ErrorCodeStatus.bit.EC01AC = 1 ;
            }
            else if( FeedBackAdcData.Status.B.EM_NoConnection == 1 )
            {
                ErrorCodeStatus.bit.EC01AF = 1 ;
            }
            else if( FeedBackAdcData.Status.B.LCB_MCU_Fail == 1 )
            {
                ErrorCodeStatus.bit.EB0247 = 1 ;
            }
            // 阻力範圍指示LED燈
            TempOffset = ADC_GetCalculatorValue(_ResistanceOffset_) ;
            if( TempOffset <= 1700 && TempOffset >= 1600 )
            {
                STATUSLED2(ON) ;
                LCBADC.ResistanceOffset = 100;
            }
            else
            {
                if( TempOffset > 1700 )
                {
                    LCBADC.ResistanceOffset = 100 + (TempOffset - 1700 ) * 20 / 1600 ;
                }
                else if(TempOffset < 1600 )
                {
                    LCBADC.ResistanceOffset = 100 - (1600 - TempOffset ) * 20 /1600 ;
                }
                STATUSLED2(OFF) ;
            }
            // 電源打開且非燒機模式
            if( LCBSystemProcessStatus.B.sConsolePowerOn == 1 && //FeedBackAdcData.Status.B.LCB_PowerOn == 1
               LCBSystemProcessStatus.B.sBurninTest == 0 )
            {
                //------------------------------------------------------------------------
                if( SendTestPWMTime > 250 )
                {
                    SendTestPWMTime = 0 ;

                    if( LCBSystemProcessStatus.B.sCutOffResistance == 1 || 
                       RPMData.AverageRPM < LCBEeprom.Member.LimitRpmForResistance )
                    {
                        SendData.EM_TestPWM = 0 ;
                        SendData.EM_CurrentCommand = 0 ;
                    }
                    else
                    {
                        LCBMain_Resistance() ;
                    }
                }                  
            }
            // 昇壓迴路HW check
            if( LCBSystemProcessStatus.B.sConsolePowerOn == 1 && _BoostHWcheck == 0)
            {// 
                if(SystemTimeCounter > 2000)
                {
                    SystemTimeCounter = 0;
                    if( LCBMain_BoostHardwareCheck() == BOOST_HW_OK )
                    {
                        
                    }
                    else
                    {
                        // Add Error Code 
                        ErrorCodeStatus.bit.EB014A = 1 ;  
                        LCBSystemProcessStatus.B.sCheckDCPlugin = 1 ;
                        LCBSystemProcessStatus.B.sCheckDCPluginOK = 0 ;
                    }
                    _BoostHWcheck = 1;
                }
            }
            //
            LCBMain_EEPROM_Process();
        }
        else
        {// ERP mode
            if( LCBMainMode != POWER_SAVE )
            {
                SystemTimeCounter = 0 ;
                LCBMainMode = POWER_SAVE ;  
            }
        }
    }
    //
}



/*******************************************************************************
* Function Name  : 
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LCBMain_Resistance(void) 
{
    unsigned short tempRPM;
    unsigned short maxWatt,machineWatt,setWatt;
    unsigned char rpmLevel,LevelMore;
    tempRPM = RPMData.AverageRPM;
    if(tempRPM > 140) tempRPM = 140;
    else if(tempRPM < 25) tempRPM = 25;
    
    rpmLevel = (tempRPM - 25) / 5;
    LevelMore = (tempRPM - 25) % 5;
    
    
#ifdef DebugEMPWM
    SendData.EM_TestPWM = ADC_GetCalculatorValue(_ResistanceOffset_) ; 
#else   
    UCBData.GeneratorWatts = Table_GeneratorCurrent2Watts(LCBADC.GeneratorCurrent,RPMData.AverageRPM);
    
    if( UCBData.SetWatts != 0 )
    {
        if(LevelMore > 0)
        {
            maxWatt = MAX_WATT[rpmLevel] + (MAX_WATT[rpmLevel + 1] - MAX_WATT[rpmLevel]) * LevelMore / 5;
            machineWatt = MACHINE_CW[rpmLevel] + (MACHINE_CW[rpmLevel + 1] - MACHINE_CW[rpmLevel]) * LevelMore / 5;
        }
        else 
        {
            maxWatt = MAX_WATT[rpmLevel];
            machineWatt = MACHINE_CW[rpmLevel];
        }       
        
        setWatt = UCBData.SetWatts;
        if( setWatt > maxWatt ) setWatt = maxWatt;
        
        UCBData.TragetWatts = LCBMain_CalculatorResistanceCommand(setWatt) ;
        SendData.EM_TestPWM = 0 ;
        UCBData.ResistanceWatts = 0;
        // Watts Control mode
        if( UCBData.TragetWatts > UCBData.GeneratorWatts + machineWatt )
        {
            UCBData.ResistanceWatts = UCBData.TragetWatts - UCBData.GeneratorWatts - machineWatt;
            if( LCBSystemProcessStatus.B.sDCPlugin == 0 && UCBData.ResistanceWatts > 15 )
            {
                if( LCBSystemProcessStatus.B.sBatteryOK == 1 && 
                   BoostControllerStatus.B.HardwareOK == 1 && 
                       BoostControllerStatus.B.OC == 0 )
                {
                    if( LCBSystemProcessStatus.B.sBatteryDischarge == 0 )
                    {
                        if( LCBSystemProcessStatus.B.sBatteryCharge == 0 )
                        {
                            if( LCBSystemProcessStatus.B.sForceBatteryCharge == 0 )
                                LCBSystemProcessStatus.B.sForceBatteryCharge = 1 ;
                        }
                        else
                        {
                            if( LCBSystemProcessStatus.B.sBatteryFloatCharge == 0 )
                            {
                                if( BatteryChargeCurrentLimit > 0 && BatteryChargeCurrentLimit < 40 )
                                {
                                    if( BatteryChargeCurrentLimit > 20)
                                        BoostOutputVoltage = 1470 ;
                                    
                                    BatteryChargeCurrentLimit += 5 ;  
                                    if( BatteryChargeCurrentLimit > 40 )
                                        BatteryChargeCurrentLimit = 40 ;
                                }
                            }
                        }
                    }
                }
            }
        }
        else
        {
            if( LCBSystemProcessStatus.B.sBatteryCharge == 1 && LCBSystemProcessStatus.B.sDCPlugin == 0 )  
            {
                if( UCBData.TragetWatts != 0 && (UCBData.TragetWatts + 15 < UCBData.GeneratorWatts + machineWatt) )
                {                                  
                    if( BatteryChargeCurrentLimit > 5  )  
                    {
                        BatteryChargeCurrentLimit -= 5 ;
                    }
                    else
                    {
                        BatteryChargeCurrentLimit = 0;
                        LCBSystemProcessStatus.B.sBatteryCharge = 0 ;
                    }
                }
            }
        }
        // ECB控制電流取得
        if( UCBData.ResistanceWatts > 0 )
        {
            SendData.EM_CurrentCommand = Table_GetElectromagnetCurrent( UCBData.ResistanceWatts,RPMData.AverageRPM );
        }
        else
            SendData.EM_CurrentCommand = 0 ;
    }
    else
    {       
#ifndef   _DisablePWM_CMD_ 
        if( UCBData.SetEMCurrent != 0 )
        {
            SendData.EM_TestPWM = 0 ;
            // Current Control Mode
            SendData.EM_CurrentCommand = UCBData.SetEMCurrent ;
        }
        else
        {// 阻力隨0x65 command設定值改變輸出
            //
            SendData.EM_CurrentCommand = 0 ;
            // Manual Mode
            if( UCBData.SetEMPWM != 0 )
            {
                SendData.EM_TestPWM = UCBData.SetEMPWM ;
            }
            else
            {
                SendData.EM_TestPWM = 0 ;
            }
        }
#else
        SendData.EM_CurrentCommand = 0 ;
        UCBData.TragetWatts = 0 ;
#endif
    }
#endif  
}


// EEPROM外掛儲存資料處理
void LCBMain_EEPROM_Process(void)
{
    
    if( CheckSaveEEPROMTime >= _SaveEepromTime )
    {// every 10ms
        CheckSaveEEPROMTime = 0 ;
        if( EepromControl.B.SaveParameter == 1 )
        {
            EepromControl.B.SaveParameter = 0 ;
            EEPROM_SaveParameter(1) ;
        }
    }
    // Log file
    if( UpdateLogFileTime >= _UpdateLogFileTime )
    {// every 1`s
        UpdateLogFileTime = 0 ;
        if( EepromControl.B.UpdateLogMax == 1 )
        {
            EepromControl.B.UpdateLogMax = 0;
            
            EepromControl.B.SaveLogMax = 1 ;
            LogMaxDataIndex = EEPROM_MaxDataReportSave(LogMaxDataIndex) ;
            EepromControl.B.SaveLogMax = 0 ;
        }
        // Check Normal file
        //-------------------------------------------------------------------
        if( RPMData.AverageRPM < 20 && RPMData.AverageRPM != 0 )
            LogFile.Count[0] += 1 ;
        else if( RPMData.AverageRPM >= 20 && RPMData.AverageRPM < 40 )
            LogFile.Count[1] += 1 ;
        else if( RPMData.AverageRPM >= 40 && RPMData.AverageRPM < 60 )
            LogFile.Count[2] += 1 ;
        else if( RPMData.AverageRPM >= 60 && RPMData.AverageRPM < 80 )
            LogFile.Count[3] += 1 ;
        else if( RPMData.AverageRPM >= 80 && RPMData.AverageRPM < 100 )
            LogFile.Count[4] += 1 ;
        else if( RPMData.AverageRPM >= 100 && RPMData.AverageRPM < 120 )
            LogFile.Count[5] += 1 ;
        else if( RPMData.AverageRPM >= 120 )
            LogFile.Count[6] += 1 ;
        //--------------------------------------------------------------------------
        if( LCBADC.GeneratorCurrent < 50 ) //0.5A		
            LogFile.Count[7] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 50 && LCBADC.GeneratorCurrent < 100 ) //0.5A~1A		
            LogFile.Count[8] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 100 && LCBADC.GeneratorCurrent < 150 ) //1A~1.5A
            LogFile.Count[9] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 150 && LCBADC.GeneratorCurrent < 200 ) //1.5A~2A
            LogFile.Count[10] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 200 && LCBADC.GeneratorCurrent < 250 ) //2A~2.5A
            LogFile.Count[11] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 250 && LCBADC.GeneratorCurrent < 300 ) //2.5A~3A
            LogFile.Count[12] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 300 && LCBADC.GeneratorCurrent < 350 ) //3A~3.5A
            LogFile.Count[13] += 1 ;
        else if( LCBADC.GeneratorCurrent >= 350 )
            LogFile.Count[14] += 1 ;
        //--------------------------------------------------------------------------
        if( LCBADC.GeneratorVoltage < 50 ) //50		
            LogFile.Count[15] += 1 ;
        else if( LCBADC.GeneratorVoltage >= 50 && LCBADC.GeneratorVoltage < 100 ) //50-100
            LogFile.Count[16] += 1 ;
        else if( LCBADC.GeneratorVoltage >= 100 && LCBADC.GeneratorVoltage < 150 ) //100~150
            LogFile.Count[17] += 1 ;
        else if( LCBADC.GeneratorVoltage >= 150 && LCBADC.GeneratorVoltage < 200 ) //150~200
            LogFile.Count[18] += 1 ;
        else if( LCBADC.GeneratorVoltage >= 200 && LCBADC.GeneratorVoltage < 250 ) //200~250
            LogFile.Count[19] += 1 ;
        else if( LCBADC.GeneratorVoltage >= 250 )
            LogFile.Count[20] += 1 ;
        //--------------------------------------------------------------------------
        if( LCBSystemProcessStatus.B.sBatteryOK == 1 )
        {
            if( LCBSystemProcessStatus.B.sBatteryDischarge == 1 )
            {
                if( LCBADC.BatteryVoltage < 1100 && LCBADC.BatteryVoltage != 0 )
                    LogFile.Count[21] += 1 ;
                else if( LCBADC.BatteryVoltage >= 1100 &&  LCBADC.BatteryVoltage < 1150)
                    LogFile.Count[22] += 1 ;
                else if(LCBADC.BatteryVoltage >= 1150 &&  LCBADC.BatteryVoltage < 1200)
                    LogFile.Count[23] += 1 ;
                else if( LCBADC.BatteryVoltage >= 1200 &&  LCBADC.BatteryVoltage < 1250 )
                    LogFile.Count[24] += 1 ;
                else if( LCBADC.BatteryVoltage >= 1250 &&  LCBADC.BatteryVoltage < 1300 )
                    LogFile.Count[25] += 1 ;
                else if( LCBADC.BatteryVoltage >= 1300 )
                    LogFile.Count[26] += 1 ;
            }
            else
            {
                if( LCBSystemProcessStatus.B.sBatteryCharge == 1 )
                {
                    if( LCBADC.BatteryChargeCurrnet < 10 && LCBADC.BatteryChargeCurrnet != 0 )
                        LogFile.Count[27] += 1 ;
                    else if( LCBADC.BatteryChargeCurrnet >= 10 && LCBADC.BatteryChargeCurrnet < 30)
                        LogFile.Count[28] += 1 ;
                    else if( LCBADC.BatteryChargeCurrnet >= 30 )
                        LogFile.Count[29] += 1 ;		
                }
            }		
        }
        //--------------------------------------------------------------------------
        if( LCBADC.ElectroMagnetCurrent < 50 )
            LogFile.Count[30] += 1 ;
        else
        {
            if( LCBADC.ElectroMagnetCurrent >= 50 &&  LCBADC.ElectroMagnetCurrent < 100 )
                LogFile.Count[31] += 1 ;
            else if(LCBADC.ElectroMagnetCurrent >= 100 &&  LCBADC.ElectroMagnetCurrent < 150)
                LogFile.Count[32] += 1 ;
            else if( LCBADC.ElectroMagnetCurrent >= 150 &&  LCBADC.ElectroMagnetCurrent < 200 )
                LogFile.Count[33] += 1 ;
            else if( LCBADC.ElectroMagnetCurrent >= 200 )
                LogFile.Count[34] += 1 ;
        }
        //
        if( LogDataSaveTime < _LogDataSaveTimeLimit )
            LogDataSaveTime += 1 ;
    }
    if( LogDataSaveTime >= _LogDataSaveTimeLimit )
    {// 當 >= 1分鐘
        if( LogDataIndex < _LogDataMaxIndex )
        {
            LogDataIndex += 1 ;
            //EEPROM_SaveRealTimeReportData(LogDataIndex) ;
            // 2014.04.21
            EEPROM_WriteParameter_C(LogDataIndex);
            //
        }
        else
        {		
            LogDataSaveTime = 0 ;
            LogDataIndex = 0 ;
        }
    }
    
}



