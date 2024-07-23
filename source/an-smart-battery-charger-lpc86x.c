/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_power.h"
#include "fsl_mrt.h"
#include "fsl_adc.h"
#include "fsl_i2c.h"
#include "fsl_ftm.h"
#include "math.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//smart battery register define
#define Temperature_Add 0x08U
#define Voltage_Add 0x09U
#define Current_Add 0x0AU
#define RemainingCapacity_Add 0x0DU
#define DischargeRemainingTime_Add 0x11U
#define ChargeRemainingTime_Add 0x13U
//period interrupt timer define
#define MRT_CLK_FREQ   CLOCK_GetFreq(kCLOCK_CoreSysClk)
//NTC temperature sensor from battery, ADC define
#define DEMO_ADC_BASE                  ADC0
#define DEMO_ADC_SAMPLE_CHANNEL_NUMBER 0U
#define DEMO_ADC_CLOCK_SOURCE          kCLOCK_Fro
#define DEMO_ADC_CLOCK_DIVIDER         1U
//I2C define
#define EXAMPLE_I2C_MASTER_BASE    (I2C0_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY CLOCK_GetMainClkFreq()
#define WAIT_TIME                  10U
#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT 0x0BU
#define I2C_BAUDRATE 100000U
#define I2C_DATA_LENGTH 3U
#define I2C_DATA_WRITE_LENGTH 2U
//Flextimer define  
/* The Flextimer base address/channel used for board */
#define BOARD_FTM_BASEADDR FTM0
#define BOARD_FTM_CHANNEL  kFTM_Chnl_0
/* Interrupt number and interrupt handler for the FTM base address used */
#define FTM_INTERRUPT_NUMBER FTM0_IRQn
#define FTM_LED_HANDLER      FTM0_IRQHandler
/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl0InterruptEnable
#define FTM_CHANNEL_FLAG             kFTM_Chnl0Flag
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_CoreSysClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif
#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (60000U)
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
//MRT
static volatile bool mrtIsrFlag          = false;
static volatile bool mrtEnableCount      = false;
static volatile uint32_t g_mrtCountValue   = 0;
static volatile uint32_t mrtDividerValue = 0;
//ADC
adc_result_info_t adcResultInfoStruct;
const uint32_t g_Adc_12bitFullRange = 4096U;
//I2C
uint8_t g_master_txBuff[I2C_DATA_LENGTH];
uint8_t g_master_rxBuff[I2C_DATA_LENGTH];
i2c_master_handle_t g_m_handle;
volatile bool g_MasterCompletionFlag = false;
//FTM
volatile bool ftmIsrFlag          = false;
volatile bool brightnessUp        = true; /* Indicate LED is brighter or dimmer */
//pwm configure
volatile uint32_t g_pwmPeriod   = 0U;
volatile uint32_t g_pulsePeriod = 0U;
volatile uint32_t g_timerClock  = 0U;
volatile uint32_t g_cycleduty_ticks   = 0U;
//polling cycles
static volatile uint32_t g_DutyDelayStage1   = 0u;
static volatile uint32_t g_DutyDelayStage2   = 0u;
//systick
volatile uint32_t g_systickCounter;
//smart battery result
uint16_t g_Voltage;
uint16_t g_Current;
uint16_t g_Temperature;
uint16_t g_RemainingCapacity;
uint16_t g_ChargeRemainingTime;
//samrt battery threshold between stages
uint16_t g_PreChargeMaxVoltage;
uint16_t g_CCChargeMaxVoltage;
uint16_t g_CVChargeMinCurrent;
//charging status
int8_t g_ChargeStatus;
typedef enum Status
{
    initial = 0U, 	/*battery is pre-charging */
    Pre_Charge , 	/*battery is pre-charging */
    CC_Charge,	 			/*battery is constant current charging */
    CV_Charge,       	/*battery is constant voltage charging */
    Charge_Full,      /*battery is charged full */
    Over_Current,	 		/*current is above 500mA */
    Over_Voltage,	 		/*voltage is above 8200mV */
    Over_Temperature,	/*temperature is above 50 */
    Disconnect,				/*deivce is disconnected */
} ChargeStatus;
uint8_t ChargesStatusString[10][30]={
  "initial","Pre_Charge","CC_Charge","CV_Charge",
  "Charge_Full","Over_Current","Over_Voltage",
  "Over Temperature","Disconnect"};
//NTC 
const float Rp = 10000.0;
const float T2 = (273.15+25.0);
const float Bx =	3950.0;
uint32_t g_Temperature_NTC;
float g_Voltage_NTC;
float Rt;
/*SPI*/
#define BUFFER_SIZE (64)
static uint8_t txBuffer[BUFFER_SIZE];
static uint8_t rxBuffer[BUFFER_SIZE];


/*******************************************************************************
 * Code
 ******************************************************************************/
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}
/*!
 * @brief read vaule of battery function
 */
uint16_t Information_Read_From_Battery(uint32_t address){
	
    status_t reVal  = kStatus_Fail;
    uint8_t deviceAddress;
    uint8_t  RxData[2];
    uint16_t ReturnData;
    i2c_master_transfer_t masterXfer = {0};
    
    deviceAddress = address;	

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kI2C_2PinOpenDrain;
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    /* Initialize the I2C master peripheral */
    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    /* Create the I2C handle for the non-blocking transfer */
    I2C_MasterTransferCreateHandle(EXAMPLE_I2C_MASTER, &g_m_handle, i2c_master_callback, NULL);

    masterXfer.slaveAddress   = I2C_MASTER_SLAVE_ADDR_7BIT;
    masterXfer.direction      = kI2C_Read;
    masterXfer.subaddress     = (uint32_t)deviceAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = RxData;
    masterXfer.dataSize       = 2;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    /* Send master non-blocking data to slave */
    reVal = I2C_MasterTransferNonBlocking(EXAMPLE_I2C_MASTER, &g_m_handle, &masterXfer);

    /*  Reset master completion flag to false. */
    g_MasterCompletionFlag = false;

    if (reVal != kStatus_Success)
    {
        return -1;
    }

    /*  Wait for transfer completed. */
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;

    ReturnData = (RxData[0]) |(RxData[1]<<8);		
    
    return(ReturnData);
}
void MRT0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    MRT_ClearStatusFlags(MRT0, kMRT_Channel_0, kMRT_TimerInterruptFlag);
    g_mrtCountValue++;
    
    if((g_mrtCountValue% 2) == 0){ //every 500ms	           
      g_Voltage = Information_Read_From_Battery(Voltage_Add);	
      g_Current = Information_Read_From_Battery(Current_Add);
//      PRINTF("\r\nCurrent: %d ,cycleduty_ticks: %d " , g_Current, g_cycleduty_ticks);		
      if((g_Voltage > g_PreChargeMaxVoltage)&&(g_ChargeStatus == Pre_Charge)){
         g_ChargeStatus = CC_Charge;
         g_DutyDelayStage1=0;
      }	
      switch (g_ChargeStatus) {
        case Pre_Charge:  
            g_cycleduty_ticks =  200;
//            PRINTF("%s \r\n" ,ChargesStatusString[Pre_Charge]);	            
            GPIO_PortClear(GPIO, 0, 1u << 24); //blue LED
            GPIO_PortSet(GPIO, 0, 1u << 25);				
            GPIO_PortSet(GPIO, 0, 1u << 26);	
            break;
        case CC_Charge:  
//            PRINTF("%s \r\n" ,ChargesStatusString[CC_Charge]);	
            if(g_Voltage > g_CCChargeMaxVoltage){
              g_ChargeStatus = CV_Charge;
            }		
            GPIO_PortClear(GPIO, 0, 1u << 24); //blue LED
            GPIO_PortSet(GPIO, 0, 1u << 25);				
            GPIO_PortSet(GPIO, 0, 1u << 26);	
            if((g_Current >= 0)&&(g_Current < 100)){			
                g_cycleduty_ticks +=  6;		
            }else if((g_Current >=100)&&(g_Current < 200)){
                g_cycleduty_ticks +=  3;	
            }else if((g_Current >=200)&&(g_Current < 340)){
                 g_cycleduty_ticks++;				
            }else if((g_Current >=340)&&(g_Current < 360)){
	                    
            }else if((g_Current >=360)&&(g_Current < 450)){
                g_cycleduty_ticks --;	                    
            }else if((g_Current >=450)&&(g_Current < 600)){
                g_cycleduty_ticks -=  3;	                    
            }else if((g_Current >=600)&&(g_Current < 700)){
                g_cycleduty_ticks -=  6;	                    
            }else if(g_Current >= 700){	
                g_ChargeStatus = Over_Current;
                g_cycleduty_ticks = 0;		
                GPIO_PortClear(GPIO, 0, 1u << 26); //red LED
                GPIO_PortSet(GPIO, 0, 1u << 24);				
                GPIO_PortSet(GPIO, 0, 1u << 25);									
            }	
            break;				
        case CV_Charge:
//            PRINTF("%s \r\n" ,ChargesStatusString[CV_Charge]);	
            if(g_Current < g_CVChargeMinCurrent){
                g_ChargeStatus = Charge_Full;
            }			
            GPIO_PortClear(GPIO, 0, 1u << 24); //blue LED
            GPIO_PortSet(GPIO, 0, 1u << 25);				
            GPIO_PortSet(GPIO, 0, 1u << 26);					
            if((g_Voltage >g_CCChargeMaxVoltage - 50)&&(g_Voltage <= g_CCChargeMaxVoltage)){					
                g_cycleduty_ticks++;
            }else if((g_Voltage >g_CCChargeMaxVoltage)&&(g_Voltage <= g_CCChargeMaxVoltage+5)){
                    
            }else if((g_Voltage >g_CCChargeMaxVoltage+5)&&(g_Voltage <= g_CCChargeMaxVoltage+50)){
                g_cycleduty_ticks--;				
            }else if(g_Voltage >g_CCChargeMaxVoltage+50){
                g_ChargeStatus = Over_Voltage;
                g_cycleduty_ticks = 0;		
                GPIO_PortClear(GPIO, 0, 1u << 26); //red LED
                GPIO_PortSet(GPIO, 0, 1u << 24);				
                GPIO_PortSet(GPIO, 0, 1u << 25);			
            }else{			
                g_cycleduty_ticks =  200;
            }	
            break;
        case Charge_Full:		
//            PRINTF("%s \r\n" ,ChargesStatusString[Charge_Full]);	
            g_ChargeStatus = Charge_Full;					
            g_cycleduty_ticks = 0;	
            GPIO_PortClear(GPIO, 0, 1u << 25); //green LED
            GPIO_PortSet(GPIO, 0, 1u << 24);				
            GPIO_PortSet(GPIO, 0, 1u << 26);				
            break;
        }	
        /* Disable channel output before updating the dutycycle */
        FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL, 0U);
        /* Update PWM duty cycle */
        if( g_cycleduty_ticks > 999){
            g_cycleduty_ticks = 999;
        }
        BOARD_FTM_BASEADDR->CONTROLS[BOARD_FTM_CHANNEL].CnV = g_cycleduty_ticks;      
        
        /* Software trigger to update registers */
        FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);
        /* Start channel output with updated dutycycle */
        FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL, FTM_PWM_ON_LEVEL);
        
        ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE);
        /* Wait for the converter to be done. */
        while (!ADC_GetChannelConversionResult(DEMO_ADC_BASE, DEMO_ADC_SAMPLE_CHANNEL_NUMBER, &adcResultInfoStruct))
        {
        }
        g_Voltage_NTC = (float)adcResultInfoStruct.result/4096*3.3;
        Rt = (33200*g_Voltage_NTC)/(3.3 - g_Voltage_NTC);
        g_Temperature_NTC = (1/(log(Rt/Rp)/Bx+(1/T2)))-273.15;	
        
        if((g_Temperature_NTC >50)&&(g_Temperature_NTC <=70)){  //over temperautre protect
            g_ChargeStatus = Over_Temperature;		
            FTM_StopTimer(BOARD_FTM_BASEADDR);
            GPIO_PortClear(GPIO, 0, 1u << 26);//red LED
            GPIO_PortSet(GPIO, 0, 1u << 25);
            GPIO_PortSet(GPIO, 0, 1u << 24);				
        }else if(g_Temperature_NTC > 70){  //over temperautre protect
            g_ChargeStatus = Disconnect;	
            FTM_StopTimer(BOARD_FTM_BASEADDR);
            GPIO_PortClear(GPIO, 0, 1u << 26);//red LED
            GPIO_PortSet(GPIO, 0, 1u << 25);
            GPIO_PortSet(GPIO, 0, 1u << 24);				
        }
    }	
    if((g_mrtCountValue% 20) == 0){  //every 2S
        g_ChargeRemainingTime = Information_Read_From_Battery(ChargeRemainingTime_Add);	
        g_RemainingCapacity   = Information_Read_From_Battery(RemainingCapacity_Add);			
        if((g_ChargeStatus == CC_Charge)||(g_ChargeStatus == CV_Charge)){                    
          PRINTF("\r\nCharging!!!Remaining time: %d min\r\n" , g_ChargeRemainingTime);	
        }else if(g_ChargeStatus == Charge_Full){
           PRINTF("\r\n%s !!!\r\n" ,ChargesStatusString[Charge_Full]);	
        }else if(g_ChargeStatus == Pre_Charge){
           PRINTF("\r\n%s !!!\r\n" ,ChargesStatusString[Pre_Charge]);	
        }else if(g_ChargeStatus == Over_Temperature){
          PRINTF("\r\n%s !!!\r\n" ,ChargesStatusString[Over_Temperature]);	
        }            
        if(g_ChargeStatus == Disconnect){                   
          PRINTF("\r\nPlease connect battery!!!\r\n");	
          PRINTF("\r\nAnd re-power the board!!!\r\n");	                   
        }else{
          PRINTF("\r\nVoltage: %d mV\r\n" , g_Voltage);
          PRINTF("\r\nCurrent: %d mA\r\n" , g_Current);	
          PRINTF("\r\nTemperature: %d Celsius\r\n" ,(uint32_t) g_Temperature_NTC);	
          PRINTF("\r\nRemaining capacity: %d%% %\r\n" ,g_RemainingCapacity);	
          PRINTF("\r\n" );			
        }					
    }	
    
    
    
    if(g_mrtCountValue > 99){
         g_mrtCountValue = 0;
    }   
}

void MRT0_Init_Config(void){
    uint32_t mrt_clock = 0;
    /* Structure of initialize MRT */
    mrt_config_t mrtConfig;
    mrt_clock = MRT_CLK_FREQ;
    /* mrtConfig.enableMultiTask = false; */
    MRT_GetDefaultConfig(&mrtConfig);
    /* Init mrt module */
    MRT_Init(MRT0, &mrtConfig);
    /* Setup Channel 0 to be repeated */
    MRT_SetupChannelMode(MRT0, kMRT_Channel_0, kMRT_RepeatMode);
    /* Enable timer interrupts for channel 0 */
    MRT_EnableInterrupts(MRT0, kMRT_Channel_0, kMRT_TimerInterruptEnable);
    NVIC_SetPriority(MRT0_IRQn, 2);

    /* Enable at the NVIC */
    EnableIRQ(MRT0_IRQn);    
    /* Start channel 0 */
    if (USEC_TO_COUNT(100000U, mrt_clock) > MRT_CHANNEL_INTVAL_IVALUE_MASK)
    {
        mrtDividerValue = 0;
        mrtEnableCount  = true;
        while (USEC_TO_COUNT((100000U >> (++mrtDividerValue)), mrt_clock) > MRT_CHANNEL_INTVAL_IVALUE_MASK)
        {
        }
        MRT_StartTimer(MRT0, kMRT_Channel_0, USEC_TO_COUNT((100000U >> mrtDividerValue), mrt_clock));
    }
    else
    {
        MRT_StartTimer(MRT0, kMRT_Channel_0, USEC_TO_COUNT(100000U, mrt_clock));
    }   
}

static void ADC_Configuration(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

/* Configure the converter. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
#endif                                                     /* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE */
    adcConfigStruct.clockDividerNumber = DEMO_ADC_CLOCK_DIVIDER;
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adcConfigStruct.resolution = kADC_Resolution12bit;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_RESOL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    adcConfigStruct.enableBypassCalibration = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    adcConfigStruct.sampleTimeNumber = 0U;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_TSAMP */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    adcConfigStruct.enableLowPowerMode = false;
#endif /* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
    adcConfigStruct.voltageRange = kADC_HighVoltageRange;
#endif /* FSL_FEATURE_ADC_HAS_TRIM_REG */
    ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
    /* Use the temperature sensor input to channel 0. */
    ADC_EnableTemperatureSensor(DEMO_ADC_BASE, true);
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */

    /* Enable channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask =
        (1U << DEMO_ADC_SAMPLE_CHANNEL_NUMBER); /* Includes channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER. */
    adcConvSeqConfigStruct.triggerMask      = 0U;
    adcConvSeqConfigStruct.triggerPolarity  = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode    = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(DEMO_ADC_BASE, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(DEMO_ADC_BASE, true); /* Enable the conversion sequence A. */
    /* Clear the result register. */
    ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE);
    while (!ADC_GetChannelConversionResult(DEMO_ADC_BASE, DEMO_ADC_SAMPLE_CHANNEL_NUMBER, &adcResultInfoStruct))
    {
    }
    ADC_GetConvSeqAGlobalConversionResult(DEMO_ADC_BASE, &adcResultInfoStruct);
}

void ADC0_Init_Config(void){ 
    /* Attach FRO clock to ADC0. */
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1U);
    /* Power on ADC0. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC) && FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC)
    uint32_t frequency = 0U;
    /* Calibration after power up. */
#if defined(FSL_FEATURE_ADC_HAS_CALIB_REG) && FSL_FEATURE_ADC_HAS_CALIB_REG
    DEMO_ADC_BASE->CTRL |= ADC_CTRL_BYPASSCAL_MASK;
    frequency = CLOCK_GetFreq(kCLOCK_BusClk);
    if (true == ADC_DoOffsetCalibration(DEMO_ADC_BASE, frequency))
#else
#if defined(SYSCON_ADCCLKDIV_DIV_MASK)
    frequency = CLOCK_GetFreq(DEMO_ADC_CLOCK_SOURCE) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
#else
    frequency = CLOCK_GetFreq(DEMO_ADC_CLOCK_SOURCE);
#endif /* SYSCON_ADCCLKDIV_DIV_MASK */
    if (true == ADC_DoSelfCalibration(DEMO_ADC_BASE, frequency))
#endif /* FSL_FEATURE_ADC_HAS_CALIB_REG */
    {
//        PRINTF("ADC Calibration Done.\r\n");
    }
    else
    {
//        PRINTF("ADC Calibration Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC_HAS_NO_CALIB_FUNC */

    /* Configure the converter and work mode. */
    ADC_Configuration();

#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    PRINTF("ADC Full Range: %d\r\n", g_Adc_12bitFullRange);
#endif /* FSL_FEATURE_ADC_HAS_CTRL_RESOL */  
}

int I2C0_Init_Config(void){
    /* Select the main clock as source clock of I2C0. */
    CLOCK_Select(kI2C0_Clk_From_MainClk);
}

uint32_t FTM_Init_Config(void){
  
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;  

    /* Fill in the FTM config struct with the default settings */
    FTM_GetDefaultConfig(&ftmInfo);
    /* Calculate the clock division based on the PWM frequency to be obtained */
    ftmInfo.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

    /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber            = BOARD_FTM_CHANNEL;
    ftmParam.level                 = pwmLevel;
    ftmParam.dutyCyclePercent      = 0;
    ftmParam.firstEdgeDelayPercent = 0U;
    ftmParam.enableComplementary   = false;
    ftmParam.enableDeadtime        = false;
    if (kStatus_Success !=
        FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
        PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
        return -1;
    }
    
    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
    
    return kStatus_Success;
}


/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */
    /* Select the main clock as source clock of USART0 (debug console) */
    CLOCK_Select(BOARD_DEBUG_USART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_BootClockFRO60M();
    BOARD_InitDebugConsole();
    PRINTF("\r\nlpc860 smart battery charger demo.\r\n");
  
    ADC0_Init_Config();//sample the value from NTC sensor
    I2C0_Init_Config();//communicate samrtbattery charger infterface
    FTM_Init_Config(); //generate PWM to trim the charge voltage
     
    g_ChargeStatus = Pre_Charge;  // the default stage is pre-charge mode.
    g_PreChargeMaxVoltage = 6200;	//6.2V, if above g_PreChargeMaxVoltage it will enter CC charge mode.
    g_CVChargeMinCurrent = 40;		//40mA, if above g_PreChargeMaxVoltage it will enter CV charge mode.
    g_CCChargeMaxVoltage = 8150;	//8.15V if above g_PreChargeMaxVoltage it will enter charge full mode.
    g_cycleduty_ticks =  200;
   
    MRT0_Init_Config();//generate a 100ms period interrupt 
 
    while (1)
    {
    }
}
