/*
 * peripherals.h
 *
 * ECE 3849 Lab 1
 * Adam Grabowski, Michael Rideout
 * Created on April 4, 2022
 *
 * Button debouncer, calibrated for 200 samples/sec
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include <stdint.h>
#include <stdbool.h>

#define BUTTON_COUNT 5				// number of buttons excluding joystick directions
#define BUTTON_AND_JOYSTICK_COUNT 9 // number of buttons including joystick directions
#define BUTTON_SAMPLES_PRESSED 2	// number of samples before a button is considered pressed
#define BUTTON_SAMPLES_RELEASED 5	// number of samples before a button is considered released

#define BUTTON_PRESSED_STATE (BUTTON_SAMPLES_RELEASED*BUTTON_SAMPLES_PRESSED) // counter value indicating button pressed state
#define BUTTON_STATE_INCREMENT (BUTTON_PRESSED_STATE/BUTTON_SAMPLES_PRESSED)
#define BUTTON_STATE_DECREMENT (BUTTON_PRESSED_STATE/BUTTON_SAMPLES_RELEASED)

#define BUTTON_SCAN_RATE 200    // [Hz] button scanning interrupt rate
#define BUTTON_INT_PRIORITY 32  // button interrupt priority (higher number is lower priority)

#define BUTTON_AUTOREPEAT_INITIAL 100   // how many samples must read pressed before autorepeat starts
#define BUTTON_AUTOREPEAT_NEXT 10       // how many samples must read pressed before the next repetition

#define JOYSTICK_UPPER_PRESS_THRESHOLD 3595     // above this ADC value, button is pressed
#define JOYSTICK_UPPER_RELEASE_THRESHOLD 3095   // below this ADC value, button is released
#define JOYSTICK_LOWER_PRESS_THRESHOLD 500      // below this ADC value, button is pressed
#define JOYSTICK_LOWER_RELEASE_THRESHOLD 1000   // above this ADC value, button is released

#define ADC_SAMPLING_RATE 1000000   // [samples/sec] desired ADC sampling rate
#define CRYSTAL_FREQUENCY 25000000  // [Hz] crystal oscillator frequency used to calculate clock rates

#define VIN_RANGE 3.3       // global voltage input range
#define PIXELS_PER_DIV 20   // determines pixel per division on the oscilloscope
#define ADC_BITS 12         // ADC has 12 bits
#define FIFO_SIZE 11        // FIFO capacity is 1 item fewer

#define ADC_BUFFER_SIZE 2048                                // size of ADC samples buffer
#define ADC_TRIGGER_SIZE 128                                // size of ADC trigger buffer
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))    // index wrapping macro

extern volatile uint32_t gButtons;	// debounced button state, one per bit in the lowest bits
extern uint32_t gJoystick[2];       // joystick coordinates
extern uint32_t gADCSamplingRate;   // [Hz] actual ADC sampling rate

extern volatile uint16_t trigger_samples[ADC_TRIGGER_SIZE]; // samples to show on LCD screen
extern volatile bool risingSlope;                           // boolean that determines slope rising or falling
extern volatile float timescale[];                          // array of floats that determine oscilloscope time scale

extern volatile uint32_t trigger_value; // equivalent to ADC offset
extern volatile int stateTperDiv;       // 12 states

// initialize all button and joystick handling hardware
void ButtonInit(void);

// update the debounced button state in the global variable gButtons
// the input argument is a bitmap of raw button state from the hardware
void ButtonDebounce(uint32_t buttons);

// sample joystick and convert to button presses
void ButtonReadJoystick(void);

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void);

// initialize ADC hardware
void ADC_Init(void);

// get CPU load count
uint32_t cpuLoadCount(void);

// search for sample trigger
int32_t risingTrigger(void);

// put data into FIFO data structure
int fifoPut(char data);

// get data from FIFO data structure
int fifoGet(char *data);

// get zero crossing point
uint32_t zeroCrossPoint(void);

#endif /* PERIPHERALS_H_ */
