import paho.mqtt.client as mqtt
import json
import time

vehicle_id = "V-001"

# This runs when the vehicle connects to the network
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"SUCCESS: Vehicle {vehicle_id} Connected.")
    # The vehicle 'subscribes' to listen for dispatch instructions from the master
    client.subscribe("lanex/charger/master/dispatch")

# This runs when the vehicle receives an instruction from the master
def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    
    # Check if the message is actually meant for this specific vehicle
    if payload['vehicle_id'] == vehicle_id:
        print(f"\n[RECEIVED INSTRUCTION] Master says: {payload['action']}")
        print(f"[ROUTING] Navigating to assigned charger: {payload['assigned_charger']}")

# --- SETUP AND RUN ---
vehicle_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=vehicle_id)
vehicle_client.on_connect = on_connect
vehicle_client.on_message = on_message

vehicle_client.connect("127.0.0.1", 1883, 60)

# Start listening in the background
vehicle_client.loop_start()

# Give it a second to connect properly
time.sleep(1)

# Create a fake battery status report
request_payload = {
  "vehicle_id": vehicle_id,
  "soc": 18, 
  "priority": "standard", 
  "location": {"lat": 20.296, "lon": 85.824}
}

print(f"\n[ACTION] Vehicle {vehicle_id} is requesting a charge...")
# Send the request to the master
vehicle_client.publish("lanex/charger/master/request", json.dumps(request_payload))

# Keep the script running for a few seconds so it can receive the reply
time.sleep(3)
vehicle_client.loop_stop()