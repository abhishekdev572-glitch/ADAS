# STM32 Nucleo — Motor & Steering Control

STM32CubeIDE project running on the Nucleo board. Handles real-time motor and steering control based on commands received from the Raspberry Pi.

## Contents

```
stm32-nucleo/
├── Core/           # Src/ and Inc/ — main application code
├── Drivers/         # HAL / CMSIS drivers (from CubeMX)
└── *.ioc           # STM32CubeMX project configuration file
```

## Hardware

- Board: _TODO (e.g., NUCLEO-F446RE)_
- Motor driver: _TODO (e.g., L298N / DRV8871)_
- Steering actuator: _TODO_

## Building & Flashing

1. Open the `.ioc` file in **STM32CubeIDE**.
2. Build the project (`Project → Build All`).
3. Flash to the Nucleo board via ST-Link (`Run → Debug` or `Run → Run`).

## Communication

Receives commands from the Raspberry Pi over [fill in: USART / UART pins used]. See [`../docs/architecture.md`](../docs/architecture.md) for the protocol.

## Features

- [ ] PWM motor speed control
- [ ] Steering actuation
- [ ] Command parsing from RPi
- [ ] [Add any safety features, e.g., emergency stop]
