/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 *
 * TempSensorMode.c
 *
 * Simple thermometer application that uses the internal temperature sensor to
 * measure and display die temperature on the segmented LCD screen
 *
 * September 2014
 * E. Chen
 *
 ******************************************************************************/

#include "AnemometerMode.h"
#include "hal_LCD.h"
#include "main.h"

                                                        // See device datasheet for TLV table memory mapping
#define CALADC_15V_30C  *((unsigned int *)0x1A1A)       // Temperature Sensor Calibration-30 C
#define CALADC_15V_85C  *((unsigned int *)0x1A1C)       // Temperature Sensor Calibration-85 C

volatile unsigned char * anemometerUnit = &BAKMEM4_H;         // Anemometer Unit
volatile unsigned short *voltageA = (volatile unsigned short *) &BAKMEM5;                          // Celsius measurement
volatile unsigned short *voltageB = (volatile unsigned short *) &BAKMEM6;

#define USE_SMCLK_ADC_TRIGGER

#ifdef USE_SMCLK_ADC_TRIGGER
    #define ADC_TRIG_CLK_FREQ 8000000
    #define SAMPLING_RATE 8000
    #define EFFECTIVE_SAMPLING_RATE 50 //should be less than sampling rate
#else
    #define ADC_TRIG_CLK_FREQ 32768
    #define SAMPLING_RATE 4096
    #define EFFECTIVE_SAMPLING_RATE 64 //should be less than sampling rate
#endif

#define DISPLAY_REFRESH_RATE 5 //HZ
#define BUFFER_SIZE 128
#define NO_OF_AVG (SAMPLING_RATE/EFFECTIVE_SAMPLING_RATE)
#define DISPLAY_REFRESH_PERIOD (2*SAMPLING_RATE/(DISPLAY_REFRESH_RATE*NO_OF_AVG))
#define EXHALE_COUNT_THRESHOLD (EFFECTIVE_SAMPLING_RATE/4)
#define FLOW_THRESHOLD 50
#define VOLTAGE_OFFSET 195.0 //0.7 V translates to 217 codes
static uint32_t voltages[2][BUFFER_SIZE];

static uint8_t head[2];
static uint8_t tail[2];
static uint8_t count[2];
static uint8_t overflow[2];

static float movingAvg[2];
static float movingFloor[2];
static float tidalVolume[2];
static float movingStd[2];
static float movingMax[2];
static float movingMin[2];
static bool displayPending[2];
static float const VOLUME_GAIN[2]={0.00120,0.0016};

inline void saveVoltage(uint8_t channelNo,uint32_t voltage)
{
    if(count[channelNo]<BUFFER_SIZE)
    {
        voltages[channelNo][head[channelNo]]=voltage;
        count[channelNo]+=1;
        head[channelNo]=(head[channelNo]+1)&(BUFFER_SIZE-1);
    }
    else
        overflow[channelNo]=1;
}

inline bool readVoltage(uint8_t channelNo,uint16_t* voltage)
{
    __disable_interrupt();
    if(count[channelNo]>0)
    {
        count[channelNo]-=1;
        __enable_interrupt();
    }
    else
    {
        __enable_interrupt();
        return false;
    }
    *voltage=voltages[channelNo][tail[channelNo]]/NO_OF_AVG;
    tail[channelNo]=(tail[channelNo]+1)&(BUFFER_SIZE-1);
    return true;
}

#ifdef USE_SMCLK_ADC_TRIGGER
// TimerA UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A1 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK
    (ADC_TRIG_CLK_FREQ/(2*SAMPLING_RATE))-1,    // Timer period.
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};

Timer_A_initCompareModeParam initCompParam =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        // Compare register 1
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable Compare interrupt
    TIMER_A_OUTPUTMODE_RESET_SET,             // Timer output mode 7
    (ADC_TRIG_CLK_FREQ/(4*SAMPLING_RATE))-1                                       // Compare value
};

#else
// TimerA UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A1 =
{
    TIMER_A_CLOCKSOURCE_ACLK,               // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK
    (ADC_TRIG_CLK_FREQ/(2*SAMPLING_RATE))-1,  // Timer comparison trigger.
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,   // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};

Timer_A_initCompareModeParam initCompParam =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,        // Compare register 1
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, // Disable Compare interrupt
    TIMER_A_OUTPUTMODE_RESET_SET,             // Timer output mode 7
    (ADC_TRIG_CLK_FREQ/(4*SAMPLING_RATE))-1                                       // Compare value
};
#endif

void calculateFlow()
{
    static const int16_t displayRefreshPeriod=DISPLAY_REFRESH_PERIOD;
    static float tempDiff;
    static bool result;
    static int16_t displayCounter=displayRefreshPeriod;
    uint16_t temp,i;
    // Turn LED1 on when waking up to calculate temperature and update display
    // Calculate Temperature in degree C and F
    //readVoltage(0,&temp);
    //readVoltage(1,&temp);
    for(i=0;i<2;i++)
    {
        result=readVoltage(i, &temp);
        if(result)
        {
            movingAvg[i]=movingAvg[i]*0.9+temp*0.1;
            tempDiff=movingAvg[i]-temp;
            if(tempDiff<0)
                tempDiff=-1*tempDiff;
            movingStd[i]=movingStd[i]*0.9+tempDiff*0.1;
            displayCounter--;
        }
    }

    if(count[0]<10)
        overflow[0]=0;
    if(count[1]<10)
        overflow[1]=0;

    if(overflow[0]|overflow[1])
        P1OUT |= BIT0;
    else
        P1OUT &= ~BIT0;
    // Update temperature on LCD
    //
    if(displayCounter<=0)
    {
       displayCounter=displayRefreshPeriod;
       displayAvg();
    }
}

void calculateTidalVolume()
{
       static const int16_t displayRefreshPeriod=DISPLAY_REFRESH_PERIOD;
       static bool result;
       static int16_t displayCounter=displayRefreshPeriod;
       static uint16_t temp,i;
       static uint8_t exhaleCount[2];
       static float tempTidalVolume[2];
       // Turn LED1 on when waking up to calculate temperature and update display
       // Calculate Temperature in degree C and F
       //readVoltage(0,&temp);
       //readVoltage(1,&temp);
       //displayEn[0]=displayEn[1]=false;
       for(i=0;i<2;i++)
       {
           result=readVoltage(i, &temp);
           if(result)
           {
               if(movingFloor[i]<temp)
                   movingFloor[i]=movingFloor[i]*0.99+temp*0.01;
               else
                   movingFloor[i]=movingFloor[i]*0.8+temp*0.2;
               displayCounter--;
               if(temp>(movingFloor[i]+FLOW_THRESHOLD))
               {
                   tempTidalVolume[i]+=(temp+VOLTAGE_OFFSET)*(temp+VOLTAGE_OFFSET)-(movingFloor[i]+VOLTAGE_OFFSET)*(movingFloor[i]+VOLTAGE_OFFSET);
                   exhaleCount[i]++;
               }
               else
               {
                   if(exhaleCount[i]>EXHALE_COUNT_THRESHOLD)
                   {
                       tidalVolume[i]=tempTidalVolume[i];
                       displayPending[i]=true;
                       //P4OUT^=BIT0;
                   }
                   tempTidalVolume[i]=0;
                   exhaleCount[i]=0;
               }
           }
       }
       if(count[0]<10)
           overflow[0]=0;
       if(count[1]<10)
           overflow[1]=0;

       if(displayPending[0] || displayPending[1])
           displayTidalVolume();
}

void inline calculate()
{
    //calculateFlow();
    calculateTidalVolume();
}


void anemometer()
{
    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use Timer trigger 1 as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */

    ADC_init(ADC_BASE,
        ADC_SAMPLEHOLDSOURCE_2,
        ADC_CLOCKSOURCE_SMCLK,
        ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input A0 and A1 for flow Sensor. A0 is sensor A, A1 is sensor B
     * Use positive reference of Internally generated Vref
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
        ADC_INPUT_VEREF_P,
        ADC_VREFPOS_AVCC,
        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
            ADC_COMPLETED_INTERRUPT);

    // Enable the Memory Buffer Interrupt
    ADC_enableInterrupt(ADC_BASE,
            ADC_COMPLETED_INTERRUPT);

    //ADC_startConversion(ADC_BASE,
    //                  ADC_REPEATED_SINGLECHANNEL);

    ADC_startConversion(ADC_BASE,
                        ADC_REPEATED_SEQOFCHANNELS);

    // Enable internal reference and temperature sensor
    PMM_enableInternalReference();
    PMM_enableTempSensor();

    // TimerA1.1 (125ms ON-period) - ADC conversion trigger signal
    Timer_A_initUpMode(TIMER_A1_BASE, &initUpParam_A1);

    //Initialize compare mode to generate PWM1
    Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

    // Start timer A1 in up mode
    Timer_A_startCounter(TIMER_A1_BASE,
        TIMER_A_UP_MODE
        );

    // Delay for reference settling
    __delay_cycles(300000);

    overflow[0]=overflow[1]=0;
    //Enter LPM3.5 mode with interrupts enabled
    while(*anemometerRunning)
    {
        //__bis_SR_register(LPM3_bits | GIE);                       // LPM3 with interrupts enabled
        //__no_operation();                                         // Only for debugger

        if (*anemometerRunning)
        {
            calculate();
        }
    }

    // Loop in LPM3 to while buttons are held down and debounce timer is running
    while(TA0CTL & MC__UP)
    {
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM3
        __no_operation();
    }

    if (*mode == ANEMOMETER_MODE)
    {
        // Disable ADC, TimerA1, Internal Ref and Temp used by TempSensor Mode
        ADC_disableConversions(ADC_BASE,ADC_COMPLETECONVERSION);
        ADC_disable(ADC_BASE);

        Timer_A_stop(TIMER_A1_BASE);

        PMM_disableInternalReference();
        PMM_disableTempSensor();
        PMM_turnOffRegulator();

        __bis_SR_register(LPM4_bits | GIE);         // re-enter LPM3.5
        __no_operation();
    }
}

void anemometerModeInit()
{
    *anemometerRunning = 1;

    movingFloor[0]=movingFloor[1]=1023;

    displayScrollText("CRADLEWISE TIDAL VOLUME METER");
    PM5CTL0 &= ~LOCKLPM5;

    RTC_stop(RTC_BASE);                           // Stop stopwatch

    // Check if any button is pressed
    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
}

void displayVal(uint8_t channel, uint16_t value)
{
    static uint8_t i=0;
    clearLCD();

    showChar('A'+channel,pos2);
    i=0;
    while(value>0)
    {
        showChar('0'+value%10,displayMap[5-i]);
        value/=10;
        i++;
    }

    //Show colon
    LCDMEM[pos2+1] |= 1<<2;

    //LCDMEM[12] = 1<<0;  //Show exclamation
    LCDMEM[12]=1<<2; //Show heart


    //showChar('M',pos6);
    //showChar('L',pos7);

    // Handle negative values
    // Negative sign
    //LCDMEM[pos1+1] |= 0x04;

    //LCDMEM[12]=0;
    //LCDMEM[12]=1<<stateLCD;


    //showChar('C',pos5);
    //showChar('C',pos6);


    // Decimal point
    //LCDMEM[pos4+1] |= 0x01;

    // Degree symbol
    //LCDMEM[pos5+1] |= 0x04;


    //LCDMEM[pos1+1] = 1; //show decimal
}

void displayAvg()
{

    static uint16_t value;
    static uint8_t i,displayChannel=0;

    //displayChannel^=1;
    displayChannel=1;

    value=(int)movingAvg[displayChannel];
    displayVal(displayChannel,value);
}

void displayStd()
{
    clearLCD();
    static uint16_t value;
    static uint8_t i,displayChannel=0;


    //displayChannel^=1;
    displayChannel=0;

    value=(int)movingStd[displayChannel];
    displayVal(displayChannel,value);
}

void displayTidalVolume()
{
    clearLCD();
    static uint16_t value;
    static uint8_t i,displayChannel=0,displayCount=0;
    static const uint8_t displaySwitchPeriod=4;

    displayCount++;

    if(displayCount<(displaySwitchPeriod/2) && displayPending[0])
        displayChannel=0;
    else
        displayChannel=1;

    if(displayCount>(displaySwitchPeriod/2-1) && displayPending[1])
        displayChannel=1;
    else
        displayChannel=0;
    displayPending[displayChannel]=false;

    value=(int)(VOLUME_GAIN[displayChannel]*tidalVolume[displayChannel]/EFFECTIVE_SAMPLING_RATE);
    if(value>100)
        displayVal(displayChannel,value);
}

void displayCount()
{
    clearLCD();
    static uint16_t value;
    static uint8_t i,displayChannel=0;


    displayChannel^=1;
    //displayChannel=0;

    displayVal(displayChannel,count[displayChannel]);
}

/*
 * ADC Interrupt Service Routine
 * Wake up from LPM3 when ADC conversion completes
 */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    static uint16_t interruptCount=0,channel;
    static uint16_t localAvgCount=0;
    static uint32_t localAvg[2];
    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            // Clear interrupt flag
            ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);
            channel=ADC_currentChannel(ADC_BASE);
            localAvg[channel]+=ADCMEM0;
            if(localAvgCount>(NO_OF_AVG*2-3))
                saveVoltage(channel,localAvg[channel]);
            localAvgCount+=1;
            if(localAvgCount>(NO_OF_AVG*2-1))
            {
                localAvgCount=0;
                localAvg[0]=localAvg[1]=0;
            }
            interruptCount+=1;
            if(interruptCount>=SAMPLING_RATE)
            {
                interruptCount=0;
                P4OUT^=BIT0;
            }
            //__bic_SR_register_on_exit(LPM3_bits);                // Exit LPM3
            break;
        default:
            break;
    }
}

