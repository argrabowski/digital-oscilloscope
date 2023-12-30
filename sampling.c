/*
 * sampling.c
 *
 * ECE 3849 Lab 1
 * Adam Grabowski, Michael Rideout
 * Created on April 4, 2022
 *
 * ECE 3849 Lab ADC handling
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "peripherals.h"

// public globals
uint32_t gADCSamplingRate;          // [Hz] actual ADC sampling rate
volatile bool risingSlope = false;   // determines whether the slope is rising or falling
volatile int stateTperDiv = 0;      // 12 states
volatile float timescale[] = {100e-3, 50e-3, 20e-3, 10e-3, 5e-3, 2e-3, 1e-3, 500e-6, 200e-6, 100e-6, 50e-6, 20e-6}; // time scales

// imported globals
extern uint32_t gSystemClock;   // [Hz] system clock frequency
extern volatile uint32_t gTime; // time in hundredths of a second

// ADC ISR globals
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1; // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // circular buffer
volatile uint32_t gADCErrors;                           // number of missed ADC deadlines
volatile uint32_t trigger_value;                        // actual trigger value

// FIFO data structure
volatile char fifo[FIFO_SIZE];  // FIFO storage array
volatile int fifo_head = 0; // index of the first item in the FIFO
volatile int fifo_tail = 0; // index one step past the last item

// initialize ADC hardware
void ADC_Init(void)
{
    // GPIO setup for analog input AIN3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    // initialize ADC1 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1;  // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor);                      // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // initialize ADC1 sampling sequence
    ADCSequenceDisable(ADC1_BASE, 0); // choose ADC1 sequence 0; disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_TIMER, 0); // specify the "timer" trigger
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END |ADC_CTL_CH3); // in the 0th step, sample channel 3 (AIN3)

    // enable interrupt, and make it the end of sequence
    ADCSequenceEnable(ADC1_BASE, 0);    // enable the sequence.  it is now sampling
    ADCIntEnable(ADC1_BASE, 0);         // enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0, 0);     // set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0);             // enable ADC1 sequence 0 interrupt in int. controller

    // initialize timer 1 for time scale
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerDisable(TIMER1_BASE, TIMER_BOTH);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerControlTrigger(TIMER1_BASE,TIMER_A,true);
    TimerLoadSet(TIMER1_BASE, TIMER_A, gSystemClock*timescale[stateTperDiv]/PIXELS_PER_DIV - 1); // Changing interval depending on the timescale
    TimerEnable(TIMER1_BASE, TIMER_BOTH); // remove load set and enable when always triggering the ADC
}

// ADC interrupt service routine
void ADC_ISR(void)
{
    ADC1_ISC_R = ADC_ISC_IN0;           // clears ADC interrupt flag
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[
           gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
           ] = ADC1_SSFIFO0_R;          // read sample from the ADC1 sequence 0 FIFO
}

// search for sample trigger
int32_t risingTrigger(void)
{
    int32_t triggerIndex = -1;
    int32_t i; // for loop

    // goes backwards through gADCBuffer array, and finds zero-crossing point index and shifts it
    for (i = ADC_BUFFER_SIZE - 1; i >= 0; i--) {
        if(risingSlope)
        {
            if((gADCBuffer[ADC_BUFFER_WRAP(i - 1)] < trigger_value) && (gADCBuffer[ADC_BUFFER_WRAP(i)] > trigger_value)) {
                triggerIndex = ADC_BUFFER_WRAP(i + 64); // 64 is half the screen width
                return triggerIndex;
            }
        }
        else
        {
            if((gADCBuffer[ADC_BUFFER_WRAP(i)] < trigger_value) && (gADCBuffer[ADC_BUFFER_WRAP(i-1)] > trigger_value)) {
                triggerIndex = ADC_BUFFER_WRAP(i + 64); // 64 is half the screen width
                return triggerIndex;
            }
        }
    }
    return 0;
}

// Returns the zero-crossing point of the ADC waveform by finding the max and min points and averaging them
uint32_t zeroCrossPoint(void)
{
    int max = 0;
    int min = 10000;

    int i;
    for (i = 0; i < ADC_BUFFER_SIZE; i++){
        if (gADCBuffer[i] > max){
            max = gADCBuffer[i];
        }

        if (gADCBuffer[i] < min){
            min = gADCBuffer[i];
        }
    }

    return (max+min)/2;
}

// put data into the FIFO, skip if full
// returns 1 on success, 0 if FIFO was full
int fifoPut(char data)
{
    int new_tail = fifo_tail + 1;
    if (new_tail >= FIFO_SIZE) new_tail = 0; // wrap around
    if (fifo_head != new_tail) {    // if the FIFO is not full
        fifo[fifo_tail] = data;     // store data into the FIFO
        fifo_tail = new_tail;       // advance FIFO tail index
        return 1;                   // success
    }
    return 0;   // full
}

// get data from the FIFO
// returns 1 on success, 0 if FIFO was empty
int fifoGet(char *data)
{
    if (fifo_head != fifo_tail) {   // if the FIFO is not empty
        *data = fifo[fifo_head];    // read data from the FIFO
        if (fifo_head + 1 >= FIFO_SIZE)
            fifo_head = 0;
        else
            fifo_head++;
        return 1;                   // success
    }
    return 0;   // empty
}

// returns the cpu load count
uint32_t cpuLoadCount(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}
