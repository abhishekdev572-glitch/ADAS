# ADAS — Advanced Driver Assistance System

A multi-board Advanced Driver Assistance System (ADAS) built for a small-scale car platform, combining computer vision, embedded motor control, and battery monitoring across three coordinated boards plus a fleet dashboard.

## System Overview

| Board | Role | Language |
|---|---|---|
| **Raspberry Pi** | Camera-based perception — lane detection, object detection, traffic sign recognition | Python |
| **STM32 Nucleo** | Real-time motor & steering control | C (STM32CubeIDE / HAL) |
| **ESP32/ESP8266** | Battery voltage/current monitoring & state-of-charge calculation | C++ (Arduino) |
| **Dashboard & Fleet Sim** | Web dashboard + MQTT-based fleet charging coordination, bridging in real battery telemetry | Python (Flask) + MQTT + HTML/JS |
