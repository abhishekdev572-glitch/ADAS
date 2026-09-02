import paho.mqtt.client as mqtt
import json
import time
import threading

# --- HARDWARE CONFIGURATION ---
VEHICLE_ID = "LaneX_Alpha"
PRIORITY = "standard"

# Starting State
state = {
    "soc": 25.0,                  # Start at 25% battery
    "status": "working",          # working, requesting, driving, charging, queued
    "lat": 20.2960,               # Fake GPS Latitude
    "lon": 85.8240,               # Fake GPS Longitude
    "assigned_charger": None
}

def on_connect(client, userdata, flags, rc, props):
    print(f"[SYSTEM] {VEHICLE_ID} Telemetry Systems Online.")
    client.subscribe("lanex/charger/master/dispatch")

def on_message(client, userdata, msg):
    global state
    payload = json.loads(msg.payload.decode())
    
    # Only listen to messages meant for this specific vehicle
    if payload.get('vehicle_id') == VEHICLE_ID:
        
        if payload['action'] == "proceed_to_charge":
            state["assigned_charger"] = payload['assigned_charger']
            state["status"] = "driving"
            print(f"\n>>> [NAVIGATION] Route set to {state['assigned_charger']}. Driving...")
            
        elif payload['action'] == "standby_in_queue":
            state["status"] = "queued"
            print(f"\n>>> [WAITING] Placed in queue position {payload['queue_position']}. Holding position.")
            
        elif payload['action'] == "abort_and_yield":
            state["status"] = "working"
            state["assigned_charger"] = None
            print(f"\n>>> [EMERGENCY] Preempted! Yielding charger and returning to work tasks.")

# --- SETUP MQTT CLIENT ---
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"{VEHICLE_ID}_Hardware")
client.on_connect = on_connect
client.on_message = on_message
client.connect("127.0.0.1", 1883, 60)
client.loop_start()

# --- THE MAIN HARDWARE LOOP ---
# This loop runs endlessly, mimicking the constant loop running on your Raspberry Pi
print("Starting Hardware Sensor Loop...")
time.sleep(2)

try:
    while True:
        # 1. DRAIN OR CHARGE THE BATTERY
        if state["status"] == "working" or state["status"] == "queued":
            state["soc"] -= 1.5  # Drain battery fast for the simulation
            # Fake GPS movement wandering around the work zone
            state["lat"] += 0.0001
            state["lon"] -= 0.0001
            
        elif state["status"] == "charging":
            state["soc"] += 5.0  # Charge up fast for the simulation
            print(f"[{VEHICLE_ID}] Charging... Battery at {state['soc']:.1f}%")

        # 2. TRIGGER CHARGE REQUEST IF LOW
        if state["soc"] <= 20.0 and state["status"] == "working":
            state["status"] = "requesting"
            print(f"\n!!! [BATTERY LOW] {state['soc']:.1f}% - Requesting charge authorization...")
            req = {"vehicle_id": VEHICLE_ID, "soc": state["soc"], "priority": PRIORITY}
            client.publish("lanex/charger/master/request", json.dumps(req))

        # 3. SIMULATE DRIVING TO THE CHARGER
        if state["status"] == "driving":
            time.sleep(2) # It takes 2 seconds to "drive" there
            state["status"] = "charging"
            print(f"\n>>> [DOCKED] Arrived at {state['assigned_charger']}. Initiating power transfer.")

        # 4. FINISH CHARGING AND RELEASE THE PAD
        if state["soc"] >= 80.0 and state["status"] == "charging":
            print(f"\n>>> [FULLY CHARGED] Target 80% reached. Disconnecting.")
            # Tell the master node this charger is now free!
            release_payload = {"charger_id": state["assigned_charger"]}
            client.publish("lanex/charger/slave/finished", json.dumps(release_payload))
            
            # Reset vehicle state
            state["status"] = "working"
            state["assigned_charger"] = None

        # 5. PUBLISH LIVE TELEMETRY
        # This is the data stream your future UI Dashboard will read!
        telemetry = {
            "vehicle_id": VEHICLE_ID,
            "soc": round(state["soc"], 1),
            "status": state["status"],
            "gps": {"lat": round(state["lat"], 5), "lon": round(state["lon"], 5)}
        }
        client.publish("lanex/fleet/telemetry", json.dumps(telemetry))

        # Wait 1 second before checking sensors again
        time.sleep(1)

except KeyboardInterrupt:
    print("Shutting down hardware mock...")
    client.loop_stop()