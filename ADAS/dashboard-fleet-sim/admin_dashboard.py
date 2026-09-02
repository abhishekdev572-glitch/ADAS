from flask import Flask, render_template, Response, request, jsonify
import paho.mqtt.client as mqtt
import json
import time

app = Flask(__name__)

system_state = {
    "vehicles": {},
    "chargers": {
        "Master_C1": {"status": "idle", "vehicle": "None", "type": "Master Node"},
        "Slave_C2": {"status": "idle", "vehicle": "None", "type": "Slave Node"},
        "Slave_C3": {"status": "idle", "vehicle": "None", "type": "Slave Node"},
        "Slave_C4": {"status": "idle", "vehicle": "None", "type": "Slave Node"}
    }
}

# --- MQTT SETUP ---
def on_connect(client, userdata, flags, rc, props):
    print("Dashboard Backend connected to Mosquitto")
    client.subscribe("lanex/fleet/telemetry")
    client.subscribe("lanex/charger/master/dispatch")
    client.subscribe("lanex/charger/slave/finished")
    client.subscribe("lanex/system/state_update") # Listen for chargers going offline

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    topic = msg.topic

    if topic == "lanex/fleet/telemetry":
        system_state["vehicles"][payload["vehicle_id"]] = payload
    
    elif topic == "lanex/charger/master/dispatch":
        if payload.get("action") == "proceed_to_charge":
            cid = payload["assigned_charger"]
            if cid in system_state["chargers"]:
                system_state["chargers"][cid]["status"] = "occupied"
                system_state["chargers"][cid]["vehicle"] = payload["vehicle_id"]
                
    elif topic == "lanex/charger/slave/finished":
        cid = payload["charger_id"]
        if cid in system_state["chargers"]:
            system_state["chargers"][cid]["status"] = "idle"
            system_state["chargers"][cid]["vehicle"] = "None"
            
    elif topic == "lanex/system/state_update":
        # Used to visually update charger offline status
        if payload.get("type") == "charger":
            cid = payload["target"]
            if cid in system_state["chargers"]:
                system_state["chargers"][cid]["status"] = payload["state"].lower()

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="LaneX_Web_Backend")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect("127.0.0.1", 1883, 60)
mqtt_client.loop_start()

# --- FLASK WEB ROUTES ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stream')
def stream():
    def event_stream():
        while True:
            yield f"data: {json.dumps(system_state)}\n\n"
            time.sleep(0.5)
    return Response(event_stream(), mimetype="text/event-stream")

# NEW: Two-way control API
@app.route('/command', methods=['POST'])
def command():
    data = request.json
    # Inject the UI command straight into the MQTT network
    mqtt_client.publish("lanex/command/override", json.dumps(data))
    return jsonify({"status": "Command dispatched to Edge Network"})

if __name__ == '__main__':
    print("Starting LaneX Admin Dashboard on http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)