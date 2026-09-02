import paho.mqtt.client as mqtt
import json
import random

chargers = {
    "Master_C1": {"status": "idle", "vehicle": None, "priority": "none"},
    "Slave_C2": {"status": "idle", "vehicle": None, "priority": "none"},
    "Slave_C3": {"status": "idle", "vehicle": None, "priority": "none"},
    "Slave_C4": {"status": "idle", "vehicle": None, "priority": "none"}
}
waiting_queue = [] 

def on_connect(client, userdata, flags, rc, props):
    print("LaneX Master Node Online. Load-Balancing Active.")
    client.subscribe("lanex/charger/master/request")
    client.subscribe("lanex/charger/slave/finished")
    client.subscribe("lanex/command/override")

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    if msg.topic == "lanex/charger/master/request":
        handle_request(client, payload)
    elif msg.topic == "lanex/charger/slave/finished":
        handle_freed_charger(client, payload)
    elif msg.topic == "lanex/command/override":
        handle_override(client, payload)

def handle_request(client, payload):
    vid = payload['vehicle_id']
    req_type = payload.get('type', 'immediate')
    priority = payload['priority']
    
    # 1. OPTIMAL PRE-BOOKING LOGIC
    if req_type == 'pre_book':
        # Find all online chargers
        valid_chargers = [c for c, d in chargers.items() if d['status'] != 'maintenance']
        if not valid_chargers:
            print(f"[ERROR] {vid} requested pre-book, but all chargers are offline!")
            return
            
        # Select a random online charger to balance the load optimally
        assigned = random.choice(valid_chargers) 
        
        print(f"[PREDICTIVE] {vid} hit 40%. Reserved {assigned} (Token expires in 60s).")
        dispatch = {
            "vehicle_id": vid, "assigned_charger": assigned, 
            "action": "pre_book_confirmed", "deadline": 60 # 60 seconds simulated time
        }
        client.publish("lanex/charger/master/dispatch", json.dumps(dispatch))
        return

    # 2. IMMEDIATE CHARGING (20% or lower)
    empty_charger = next((c for c, d in chargers.items() if d['status'] == 'idle'), None)
    
    if empty_charger:
        assign_charger(client, vid, empty_charger, priority)
    else:
        if priority == "high":
            preemptable = next((c for c, d in chargers.items() if d['priority'] == 'standard'), None)
            if preemptable:
                kicked_vid = chargers[preemptable]['vehicle']
                print(f"[PREEMPTION] Aborting {kicked_vid} to make room for {vid}!")
                client.publish("lanex/charger/master/dispatch", json.dumps({"vehicle_id": kicked_vid, "action": "abort_and_yield"}))
                waiting_queue.insert(0, {"vehicle_id": kicked_vid, "priority": "standard"})
                assign_charger(client, vid, preemptable, priority)
                return

        waiting_queue.append({"vehicle_id": vid, "priority": priority})
        client.publish("lanex/charger/master/dispatch", json.dumps({"vehicle_id": vid, "action": "standby_in_queue"}))

def handle_freed_charger(client, payload):
    cid = payload['charger_id']
    if chargers[cid]['status'] != 'maintenance':
        chargers[cid] = {"status": "idle", "vehicle": None, "priority": "none"}
        if waiting_queue:
            next_veh = waiting_queue.pop(0)
            assign_charger(client, next_veh['vehicle_id'], cid, next_veh['priority'])

def handle_override(client, payload):
    target = payload.get("target")
    state = payload.get("state").lower()
    
    # BUG FIX: If UI sends "active", we need to set the backend to "idle" so it accepts cars again.
    if state == "active":
        state = "idle"
        
    if target in chargers:
        chargers[target]['status'] = state
        print(f"[SYS-ADMIN] {target} forced into {state.upper()}")
        client.publish("lanex/system/state_update", json.dumps({"type": "charger", "target": target, "state": state}))

def assign_charger(client, vid, cid, priority):
    chargers[cid]['status'] = "occupied"
    chargers[cid]['vehicle'] = vid
    chargers[cid]['priority'] = priority
    dispatch = {"vehicle_id": vid, "assigned_charger": cid, "action": "proceed_to_charge"}
    client.publish("lanex/charger/master/dispatch", json.dumps(dispatch))

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="LaneX_Master")
client.on_connect = on_connect
client.on_message = on_message
client.connect("127.0.0.1", 1883, 60)
client.loop_forever()