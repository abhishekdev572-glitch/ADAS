# Raspberry Pi — Perception & Control

Python code running on the Raspberry Pi. Captures camera frames, runs lane detection and object detection (YOLO), fuses in IMU data, and sends steering/speed commands to the STM32 Nucleo over I2C.

## Contents

```
raspberry-pi/
├── src/
│   └── lane20_1_.py     # Main perception + control script
├── models/               # YOLO weights go here (not committed if large — see .gitignore)
└── requirements.txt
```

## What it does (`lane20_1_.py`)

- **Lane detection** via OpenCV, with a finite state machine (`KEEP_LANE`, `CHANGE_LEFT`, `CHANGE_RIGHT`, `EMERGENCY_STOP`).
- **Object detection** using YOLO (Ultralytics).
- **IMU fusion**: reads an ISM330DHCX (accel + gyro) over I2C and applies a complementary filter (`CF_ALPHA`) to estimate orientation, with a calibration routine at startup.
- **Steering control**: PD controller (`KP`, `KD`) with IMU yaw-rate feed-forward (`KP_YAW`), steering angle/step limits (`MAX_STEER`, `MAX_STEER_STEP`).
- **I2C link to STM32 Nucleo**: sends commands to the Nucleo at address `0x12` (see [`../docs/architecture.md`](../docs/architecture.md)).

Key tunable parameters (speed, PD gains, lane-change thresholds) are defined near the top of the file.

## Setup

```bash
cd raspberry-pi
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

`picamera2` and `smbus2` require a Raspberry Pi with camera and I2C enabled (`raspi-config` → Interface Options).

## Running

```bash
python3 src/lane20_1_.py
```

## Hardware Requirements

- Raspberry Pi (with camera module, Picamera2-compatible)
- ISM330DHCX IMU (I2C, address `0x6B`)
- I2C connection to STM32 Nucleo (address `0x12`)
