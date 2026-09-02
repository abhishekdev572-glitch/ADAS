# Battery Monitor — Cargo A Node

> **Note:** despite the folder name (kept for consistency with the rest of the project), this board runs on the **ESP8266** platform (`ESP8266WiFi.h`), not ESP32. If you're actually running an ESP32 elsewhere, let me know and I'll rename/split accordingly.

Firmware that reads the car's battery voltage/current via an INA219 sensor, calculates state of charge (SoC) and estimated discharge time, and streams the data over UDP (signed with HMAC) to the [Dashboard & Fleet Sim](../dashboard-fleet-sim/README.md), which listens for it in `fleet_swarm.py`.

## Contents

```
esp32-battery-monitor/
└── src/
    ├── battery_monitor.ino     # Main sketch
    └── secrets.h.example        # Template for WiFi/secret config — copy to secrets.h
```

## Hardware

- Board: ESP8266 (e.g. NodeMCU / Wemos D1 Mini — confirm exact model)
- Sensor: Adafruit INA219 (I2C, wired to `D2`/`D1`)
- Battery: 2S Li-ion/LiPo pack (`minVoltage = 6.4V`, `maxVoltage = 8.4V`, capacity `2.2 Ah`)

## Setup

1. Install libraries via Arduino IDE Library Manager: **Adafruit INA219**, **ESP8266WiFi** (bundled with ESP8266 board package).
2. Copy `secrets.h.example` → `secrets.h` and fill in:
   ```cpp
   const char* ssid = "...";
   const char* password = "...";
   const char* secretKey = "...";
   ```
   `secrets.h` is gitignored and will never be committed.
3. Set `serverIP`/`serverPort` in `battery_monitor.ino` to the machine running `fleet_swarm.py` (default port `5005`).
4. Upload `battery_monitor.ino` to the board.

## How it works

- Reads bus voltage, current, and power from the INA219 every 5 seconds.
- Computes **SoC** as a linear map of voltage between `minVoltage` and `maxVoltage` (0–100%).
- Computes **estimated discharge time** (hours) from remaining capacity ÷ current draw.
- Builds a JSON payload (`soc`, `power`, `discharge_time`), signs it with a SHA-1 HMAC-style signature (`sig`) over the payload + `secretKey`, and sends it via UDP to the dashboard/fleet-sim server.

## ⚠️ Security note

- **Rotate the WiFi password and secret key** that were previously hardcoded in this file before making the repo public — they were exposed in earlier versions of this code and should be treated as compromised.
- The signature scheme here is a simple `sha1(payload + secretKey)` — fine for a course project, but note it's not a standard HMAC construction (no proper key/message separation) if you want to harden it later.
