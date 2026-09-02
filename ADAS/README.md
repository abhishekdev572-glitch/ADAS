# ADAS — Advanced Driver Assistance System

A multi-board Advanced Driver Assistance System (ADAS) built for a small-scale car platform, combining computer vision, embedded motor control, and battery monitoring across three coordinated boards.

## System Overview

| Board | Role | Language |
|---|---|---|
| **Raspberry Pi** | Camera-based perception — lane detection, object detection, traffic sign recognition | Python |
| **STM32 Nucleo** | Real-time motor & steering control | C (STM32CubeIDE / HAL) |
| **ESP32** | Battery voltage/current monitoring & state-of-charge calculation (actually runs on ESP8266 — see module README) | C++ (Arduino) |
| **Dashboard & Fleet Sim** | Web dashboard + MQTT-based fleet charging coordination, bridging in real battery telemetry | Python (Flask) + MQTT + HTML/JS |

```
 ┌────────────────┐   I2C   ┌──────────────────┐        ┌───────────────┐
 │  Raspberry Pi   ├────────►│   STM32 Nucleo    │        │     ESP32      │
 │  (Perception)   │         │  (Motor Control)  │        │ (Battery Mon.) │
 │  Python + OpenCV│         │  Steering/Throttle│        │  Voltage/Curr. │
 └────────────────┘         └──────────────────┘        └───────┬───────┘
                                                                    │ UDP
                                                          ┌─────────▼─────────┐
                                                          │ Dashboard/Fleet Sim │
                                                          │ Flask + MQTT + UI    │
                                                          └────────────────────┘
```

See [`docs/architecture.md`](docs/architecture.md) for the full data-flow and wiring details.

## Repository Structure

```
ADAS/
├── raspberry-pi/            # Vision & decision-making code (Python)
├── stm32-nucleo/             # Motor & steering firmware (STM32CubeIDE project)
├── esp32-battery-monitor/    # Battery calculation firmware (Arduino/PlatformIO)
├── dashboard-fleet-sim/       # Web dashboard + MQTT fleet charging simulation
├── docs/                     # Architecture notes, diagrams, wiring
└── media/                    # Demo photos/videos
```

## Subsystems

- **[Raspberry Pi — Perception](raspberry-pi/README.md)**: camera input, lane detection, object/sign detection, sends control commands to the Nucleo.
- **[STM32 Nucleo — Motor Control](stm32-nucleo/README.md)**: receives commands and drives motor/steering actuators in real time.
- **[ESP32 — Battery Monitor](esp32-battery-monitor/README.md)**: measures battery voltage/current and calculates remaining charge/runtime.
- **[Dashboard & Fleet Sim](dashboard-fleet-sim/README.md)**: Flask + MQTT dashboard visualizing vehicle/charger status, bridging in real battery telemetry alongside simulated fleet vehicles.

## Getting Started

Each subsystem has its own setup instructions — see the README inside each folder.

## Status

🚧 Work in progress — components are being integrated and documented.

## License

See [LICENSE](LICENSE).
