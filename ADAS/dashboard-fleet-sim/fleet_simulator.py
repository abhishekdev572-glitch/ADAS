import paho.mqtt.client as mqtt
import json
import time
import threading

def simulate_vehicle(vid, priority, delay_before_request):
    time.sleep(delay_before_request) # Wait a bit before requesting
    
    def on_connect(client, userdata, flags, rc, props):
        client.subscribe("lanex/charger/master/dispatch")

    def on_message(client, userdata, msg):
        payload = json.loads(msg.payload.decode())
        if payload['vehicle_id'] == vid:
            if payload['action'] == "proceed_to_charge":
                print(f"[{vid}] Received clear to charge at {payload['assigned_charger']}! Driving there now.")
            elif payload['action'] == "standby_in_queue":
                print(f"[{vid}] Placed in queue. Position: {payload['queue_position']}. Continuing logistics tasks...")
            elif payload['action'] == "abort_and_yield":
                print(f"[{vid}] EMERGENCY OVERRIDE RECEIVED! Aborting charge and yielding pad.")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=vid)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("127.0.0.1", 1883, 60)
    client.loop_start()

    request_payload = {
        "vehicle_id": vid,
        "soc": 15,
        "priority": priority
    }
    
    print(f"\n---> [{vid}] ({priority.upper()} priority) broadcasting charge request...")
    client.publish("lanex/charger/master/request", json.dumps(request_payload))
    
    time.sleep(10) # Keep vehicle alive to listen for messages
    client.loop_stop()

# --- RUN THE FLEET SIMULATION ---
print("Starting Fleet Simulation in 2 seconds...")
time.sleep(2)

# Spawn 3 standard vehicles 1 second apart
threading.Thread(target=simulate_vehicle, args=("Standard_V1", "standard", 0)).start()
threading.Thread(target=simulate_vehicle, args=("Standard_V2", "standard", 1)).start()
threading.Thread(target=simulate_vehicle, args=("Standard_V3", "standard", 2)).start()

# Wait 4 seconds, then spawn a High Priority Emergency Vehicle
threading.Thread(target=simulate_vehicle, args=("AMBULANCE_01", "high", 4)).start()