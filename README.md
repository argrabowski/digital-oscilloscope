# Digital Oscilloscope

## Project Overview

In this lab, a real-time digital oscilloscope capable of capturing and displaying waveforms on an LCD is implemented. The oscilloscope features adjustable voltage and time scales, trigger settings, and real-time CPU load measurement. The core components of the project include ADC sampling, waveform display, trigger functionality, and user input processing.

## Features

- **Adjustable Time and Voltage Scales:**
  - The oscilloscope supports multiple time and voltage scales for flexible waveform visualization.

- **Triggering:**
  - Users can set trigger points and trigger slopes (rising or falling) for accurate waveform capture.

- **Real-Time CPU Load Measurement:**
  - The system continuously measures and displays the CPU load, providing insights into system performance.

- **User Input Handling:**
  - Button presses are processed to adjust voltage and time scales, trigger settings, and other parameters.
    - Press **button 1** to change the time scale.
    - Press **button 2** to change the trigger slope.
    - Press **BoosterPack button 1** to change the voltage scale.

## Hardware Requirements

- EK-TM4C1294XL LaunchPad
- BOOSTXL-EDUMKII BoosterPack

## Getting Started

To run this project on your hardware, follow these steps:

1. Clone the repository to your local machine.
2. Open the project in the appropriate development environment for the TM4C1294 microcontroller.
3. Configure the project settings and build the code.
4. Flash the compiled binary onto the EK-TM4C1294XL LaunchPad.
5. Power on the hardware and observe the oscilloscope functionality on the LCD.
