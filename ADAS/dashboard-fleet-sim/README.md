# Dashboard & Fleet Charging Simulation

A Flask + MQTT (Mosquitto) system that simulates and visualizes fleet-level EV charging coordination — vehicles requesting charge slots, a master node load-balancing across chargers, and a live web dashboard showing status.

This module also bridges in **real hardware telemetry** (battery data) from the actual car over UDP, alongside simulated fleet vehicles — see `fleet_swarm.py`.

## Contents

```
dashboard-fleet-sim/
├── master_node.py         # Charger load-balancer / dispatcher (MQTT)
├── admin_dashboard.py      # Flask backend serving the live dashboard
├── fleet_swarm.py          # Swarm sim + UDP listener for real hardware telemetry
├── fleet_simulator.py      # Simulates a single vehicle requesting/receiving charge dispatch
├── hardware_mock.py         # Mocks one vehicle's telemetry (SoC, GPS, status) over MQTT
├── test_vehicle.py          # Minimal test client for the MQTT dispatch flow
├── dashboard.html           # Dashboard UI (standalone)
├── templates/index.html     # Dashboard UI (Flask-served)
├── static/                  # Dashboard assets (map, camera feed images)
├── paho-mqtt.js              # MQTT JS client for the browser dashboard
├── 1_Setup_Network.bat        # Windows: sets up Mosquitto broker config
└── 2_Launch_Ecosystem.bat     # Windows: launches the full simulation stack
```

## How it works

- **`master_node.py`** subscribes to charger request/finished/override topics over MQTT and assigns vehicles to chargers (`Master_C1`, `Slave_C2/3/4`), handling pre-booking, priority preemption, and a waiting queue.
- **`fleet_swarm.py` / `fleet_simulator.py` / `hardware_mock.py` / `test_vehicle.py`** simulate one or more vehicles publishing charge requests and telemetry (state of charge, GPS, status).
- **`fleet_swarm.py`** also runs a UDP listener to receive real telemetry (SoC, power, discharge time) from actual car hardware (e.g. the ESP32 battery monitor), merging simulated and real data.
- **`admin_dashboard.py`** is a Flask app that serves the live dashboard (`templates/index.html`) showing vehicle/charger status in real time over MQTT.

## Setup

Requires an MQTT broker — [Mosquitto](https://mosquitto.org/) — running locally.

```bash
pip install flask paho-mqtt
```

**Windows** (as originally set up): run `1_Setup_Network.bat` to configure/start Mosquitto, then `2_Launch_Ecosystem.bat` to launch the dashboard + nodes.

**Manual / cross-platform:**
```bash
mosquitto -v                 # start the broker
python master_node.py         # start the load-balancer
python admin_dashboard.py     # start the dashboard (Flask)
python fleet_swarm.py         # start the fleet simulation + real-hardware UDP bridge
```

Then open the dashboard in your browser at the address Flask prints (default `http://localhost:5000`).

## Notes

- `dashboard.html` appears to be a standalone/earlier version of the UI; `templates/index.html` is the one served by Flask.
- Update broker host/port and MQTT topics in each script if not running everything on `localhost`.
