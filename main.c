/*
 * main.c
 *
 * ECE 3849 Lab 1
 * Adam Grabowski, Michael Rideout
 * Created on April 4, 2022
 *
 * EK-TM4C1294XL LaunchPad with BOOSTXL-EDUMKII BoosterPack
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sysctl_pll.h"
#include "peripherals.h"

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz

uint32_t gSystemClock; // [Hz] system clock frequency
const char * const gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", "1 V", "2 V"};
const char * const gTimeScaleStr[] = {"100 ms", "50 ms", "20 ms", "10 ms", "5 ms", "2 ms", "1 ms", "500 us", "200 us", "100 us", "50 us", "20 us"};
const char * const gTriggerSlopeStr[] = { "Falling","Rising"};

// ADC ISR globals
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // circular buffer
volatile uint16_t trigger_samples[ADC_TRIGGER_SIZE]; // signal samples of trigger

uint32_t countUnloaded = 0;    // CPU count unloaded
uint32_t countLoaded = 0;      // CPU count loaded
float cpuLoad = 0.0;           // CPU load value

// enable Floating Point Unit
void FPUEnable();

// permit ISRs to use Floating Point Unit
void FPULazyStackingEnable();

// initialize singal source
void signalInit(void);

int main(void)
{
    IntMasterDisable(); // globally disable interrupts

    FPUEnable();                // enable Floating Point Unit
    FPULazyStackingEnable();    // permit ISRs to use Floating Point Unit

    // initialize system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);  // initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);      // select font

    char tscale_str[50];    // time string buffer for time scale
    char vscale_str[50];    // time string buffer for voltage scale
    char tslope_str[50];    // time string buffer for trigger edge
    char cpu_str[50];       // time string buffer for cpu load
    char bpresses[10];      // holds fifo button presses

    uint32_t stateVperDiv = 4;                      // 5 states
    uint32_t y;                                     // holds the converted ADC samples
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};   // array of voltage scale per division

    // full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    ButtonInit();   // initialize all button and joystick handling hardware
    ADC_Init();     // initialize ADC hardware

    countUnloaded = cpuLoadCount(); // get initial cpu load count

    IntMasterEnable(); // globally enable interrupts

    signalInit(); // initialize signal source

    while (true) {
        trigger_value = zeroCrossPoint(); // finds ADC_OFFSET

        // CPU Load calculations
        countLoaded = cpuLoadCount();
        cpuLoad = 1.0f - (float)countLoaded/countUnloaded; // compute CPU load

        int i; // for loop iterations
        float fScale = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * fVoltsPerDiv[stateVperDiv]); // determines fScale

        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);         // fill screen with black
        GrContextForegroundSet(&sContext, ClrWhite);    // yellow text
		
        int32_t triggerIndex = risingTrigger(); // search for sample trigger

        // local buffer retrieves 128 samples of the gADCBuffer from the trigger_index previously found
        for (i = 0; i < ADC_TRIGGER_SIZE; i++) {
            trigger_samples[i] = gADCBuffer[ADC_BUFFER_WRAP(triggerIndex - (ADC_TRIGGER_SIZE - 1) + i)];
        }
		
        // handle button presses
        if (fifoGet(bpresses)) {
            // read bpresses and change state based on bpresses buttons and whether it is being pressed currently
            for (i = 0; i < 10; i++){
                if (bpresses[i]==('u') && gButtons == 4){ // increment state
                    stateVperDiv = (++stateVperDiv) % 5;
                } else if (bpresses[i]==('t') && gButtons == 2){ // change trigger
                    risingSlope = !risingSlope;
                } else if (bpresses[i]==('h') && gButtons == 1){ // change time scale
                    stateTperDiv = (++stateTperDiv) % 12;
                }
            }
        }

        // draw grid in blue
        GrContextForegroundSet(&sContext, ClrBlue); // blue context
        for (i = 1; i < 128; i+=21){
            GrLineDraw(&sContext, i, 0, i, 128);
            GrLineDraw(&sContext, 0, i, 128, i);
        }

        // draw center grid lines in dark blue
        GrContextForegroundSet(&sContext, ClrDarkBlue); // dark blue context
        GrLineDraw(&sContext, 64, 0, 64, 128);
        GrLineDraw(&sContext, 0, 64, 128, 64);

        // draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow context
        int x;
        int y_old;
        for (x = 0; x < LCD_HORIZONTAL_MAX - 1; x++) {
            y = ((int)(LCD_VERTICAL_MAX/2) - (int)(fScale*(int)(trigger_samples[x] - trigger_value)));
            if (x!=0)
                GrLineDraw(&sContext, x-1, y_old, x, y);
            y_old = y;
        }

        // time scale, voltage scale, trigger slope and CPU load
        GrContextForegroundSet(&sContext, ClrWhite); // white context
        snprintf(tscale_str, sizeof(tscale_str), gTimeScaleStr[stateTperDiv]); // convert time to string
        GrStringDraw(&sContext, tscale_str, /*length*/ -1, /*x*/ 7, /*y*/ 5, /*opaque*/ false);

        snprintf(vscale_str, sizeof(vscale_str), gVoltageScaleStr[stateVperDiv]);
        GrStringDraw(&sContext, vscale_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 - 20, /*y*/ 5, /*opaque*/ false);

        snprintf(tslope_str, sizeof(tslope_str), gTriggerSlopeStr[risingSlope]);
        GrStringDraw(&sContext, tslope_str, /*length*/ -1, /*x*/ LCD_HORIZONTAL_MAX/2 + 20, /*y*/ 5, /*opaque*/ false);

        snprintf(cpu_str, sizeof(cpu_str), "CPU Load: %.3f%c", cpuLoad*100, 37);
        GrStringDraw(&sContext, cpu_str, /*length*/ -1, /*x*/ 5, /*y*/ LCD_VERTICAL_MAX-10, /*opaque*/ false);

        GrFlush(&sContext); // flush the frame buffer to the LCD
    }
}

// initialize singal source
void signalInit(void)
{
    // configure M0PWM2, at GPIO PF2, which is BoosterPack 1 header C1 (2nd from right) pin 2
    // configure M0PWM3, at GPIO PF3, which is BoosterPack 1 header C1 (2nd from right) pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // PF2 = M0PWM2, PF3 = M0PWM3
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f)); // 40% duty cycle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}
