# Architecture

## Boards & Responsibilities

### Raspberry Pi (Perception)
- Captures video from an onboard camera.
- Runs lane detection, object detection, and traffic sign recognition.
- Converts perception output into control decisions (e.g., steer left/right, slow down, stop).
- Sends these decisions to the STM32 Nucleo over [fill in: UART / CAN / I2C / Serial-over-USB].

### STM32 Nucleo (Motor Control)
- Receives control commands from the Raspberry Pi.
- Drives motor and steering actuators via [fill in: PWM / motor driver IC, e.g. L298N/DRV8871].
- Runs any low-level closed-loop control (e.g., PID for speed/steering).

### ESP32 (Battery Monitoring)
- Reads battery voltage and current via [fill in: voltage divider / INA219/INA226 sensor].
- Calculates remaining charge / estimated runtime (state of charge).
- Reports telemetry to the **Dashboard & Fleet Sim** module over UDP (`fleet_swarm.py`'s UDP listener, port 5005), which merges it with simulated fleet vehicle data for the live dashboard.

### Dashboard & Fleet Sim (Flask + MQTT)
- `master_node.py` load-balances charging requests across chargers via MQTT.
- `fleet_swarm.py` receives real battery telemetry from the ESP32 over UDP and merges it with simulated fleet vehicles.
- `admin_dashboard.py` serves a live web dashboard (Flask) showing vehicle/charger status.

## Data Flow

```
Camera → RPi (vision + decision) → STM32 (motor/steering actuation)

ESP32 (battery data) → UDP → Dashboard/Fleet Sim (fleet_swarm.py) → MQTT → Dashboard UI
```

## Wiring / Pinout

_TODO: add pin mapping between boards (e.g., RPi TX/RX ↔ Nucleo USART pins, ESP32 sensor pins) and a photo/diagram of the physical wiring._

## Communication Protocol

_TODO: describe the message format between RPi ↔ Nucleo (e.g., simple ASCII commands like `L`, `R`, `S`, or a defined packet structure)._
