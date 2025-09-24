# UARTBidirectional_NucleoSTM32F0Series_PlatformIO

Bare-metal demo for STM32F072RB (Nucleo board) implementing **bidirectional UART** with:
- **Interrupt-driven RX**
- **Circular buffer** for reliability
- **LED control via keyboard input**

## Features
- USART2 on PA2 (TX) and PA3 (RX)
- Circular buffer for received data (avoids data loss)
- Interrupt handler using NVIC
- Input: characters from keyboard
- Output: echo to terminal
- Action: 'a' turns PA5 LED on, other keys turn LED off

## Hardware
- Board: Nucleo-F072RB
- UART2 pins: PA2 (TX), PA3 (RX)
- Built-in LED: PA5

## Build
```bash
pio run
