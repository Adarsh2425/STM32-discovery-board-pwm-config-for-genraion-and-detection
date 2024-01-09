# Software Overview

## Introduction

This document provides an in-depth explanation of the software components and functionalities in the STM32 Discovery Board PWM Generation and Detection project.

## Code Structure

The project's code is designed to accomplish PWM signal generation and detection. Here's a detailed breakdown of key components and functionalities:

### Global Variables

- `risingEdgeTime` and `fallingEdgeTime`: These variables store the timestamp of rising and falling edges during PWM signal detection.
- `dutyCycle`: Represents the calculated duty cycle of the incoming PWM signal.
- `frequency`: Holds the calculated frequency of the incoming PWM signal.

### PWM Generation (Timer 1 - MX_TIM1_Init)

The PWM signal generation is configured using Timer 1 (`TIM1`). Here are the essential configurations:

- **Prescaler and Period:** The prescaler (`htim1.Init.Prescaler`) and period (`htim1.Init.Period`) settings determine the PWM signal frequency.
- **CCR1:** The value set in `TIM1->CCR1` adjusts the duty cycle of the PWM signal.

### PWM Detection (Timer 2 - MX_TIM2_Init)

Timer 2 (`TIM2`) is utilized for capturing and analyzing the incoming PWM signal. Key configurations include:

- **Input Capture Configuration:** `HAL_TIM_IC_ConfigChannel` sets up the input capture feature for detecting rising and falling edges.

### Input Capture Callback (HAL_TIM_IC_CaptureCallback)

The callback function is triggered when a capture event occurs on Timer 2 (`TIM2`). It calculates the duty cycle, frequency, and other parameters.

### UART Communication (MX_USART1_UART_Init)

UART communication is established through USART1 (`huart1`) for outputting results to the console.

### Main Loop (main)

The main loop of the program is structured to simulate LED behavior (flashing) and continuously monitor the PWM signal.

## Usage

1. **Connect PWM Signal:**
   - Connect the external PWM signal to the designated input pin.

2. **Run the Code:**
   - Flash the code onto the STM32 Discovery board.

3. **Monitor Results:**
   - View duty cycle and frequency values in the console.

## User Configuration

Adjust the following parameters in the code to match specific requirements:

- **PWM Signal Pin Configurations:** Modify GPIO pin configurations for PWM input.
- **Timer Settings:** Adjust prescaler and period based on the desired PWM frequency.
- **Additional GPIO Configurations:** Add or modify GPIO configurations or features, if needed.

## Dependencies

- **STM32CubeIDE:** The project is developed using STM32CubeIDE, offering a comprehensive development environment for STM32 microcontrollers.

## Additional Notes

Feel free to explore and modify the code based on your project's needs. Detailed comments are provided within the code for better understanding. For more details, refer to the [main README](README.md).

Happy coding! 🚀
