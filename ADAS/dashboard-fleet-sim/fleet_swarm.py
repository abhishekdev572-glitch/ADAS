import paho.mqtt.client as mqtt
import json
import time
import threading
import random
import socket
import hashlib
from datetime import datetime, timedelta

# --- SHARED MEMORY FOR REAL HARDWARE DATA ---
cargo_a_real_telemetry = {
    "soc": None,
    "power": 0.0,
    "discharge_time": 0.0,
    "last_seen": 0  # Initializes at 0 (Offline)
}

# --- SECURE UDP SERVER (DIGITAL TWIN LINK) ---
def udp_listener():
    UDP_PORT = 5005 
    SECRET_KEY = "CargoA_Secret_Key_2025"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
    
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except AttributeError:
        pass 
        
    sock.bind(("", UDP_PORT)) 
    
    print(f"[EDGE-LINK] Digital Twin Server Active on Port {UDP_PORT}")
    print(f"[EDGE-LINK] Waiting for ESP8266 Subnet Broadcasts...")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode('utf-8').strip()
            print(f"\n[EDGE-LINK] INBOUND from {addr[0]}: {msg}") 
            
            if ',"sig":"' in msg:
                payload_str, sig_part = msg.rsplit(',"sig":"', 1)
                payload_for_verification = payload_str + '}' 
                sig_received = sig_part.rstrip('"}')
                
                hash_input = payload_for_verification + SECRET_KEY
                sig_calc = hashlib.sha1(hash_input.encode('utf-8')).hexdigest()
                
                if sig_calc == sig_received:
                    parsed = json.loads(payload_for_verification)
                    cargo_a_real_telemetry["soc"] = float(parsed.get("soc", 0.0))
                    cargo_a_real_telemetry["power"] = float(parsed.get("power", 0.0))
                    cargo_a_real_telemetry["discharge_time"] = float(parsed.get("discharge_time", 0.0))
                    cargo_a_real_telemetry["last_seen"] = time.time() # Updates the Heartbeat!
                else:
                    print(f"[EDGE-LINK] SECURITY ALERT: Invalid Hash!")
        except Exception as e:
            pass

# --- FLEET SIMULATION LOGIC ---
class FleetVehicle:
    def __init__(self, vid, priority):
        self.vid = vid
        self.priority = priority
        self.soc = random.uniform(30.0, 100.0) 
        self.payload_kg = random.randint(0, 2000) 
        self.status = "working" 
        
        self.lat = 20.2960 + random.uniform(-0.01, 0.01)
        self.lon = 85.8240 + random.uniform(-0.01, 0.01)
        self.target_lat = self.lat
        self.target_lon = self.lon
        self.get_new_waypoint()
        
        self.charger = None
        self.pre_booked_charger = None
        self.deadline = None 
        self.idle_timer = 0
        
        self.charge_cycles = random.randint(12, 150)
        self.health_score = random.uniform(88.0, 99.9)
        self.last_maint = (datetime.now() - timedelta(days=random.randint(10, 60))).strftime("%d %b %Y")
        self.next_maint = (datetime.now() + timedelta(days=random.randint(5, 30))).strftime("%d %b %Y")
        
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sim_{self.vid}")
        self.client.on_message = self.on_message
        self.client.connect("127.0.0.1", 1883, 60)
        self.client.subscribe("lanex/charger/master/dispatch")
        self.client.subscribe("lanex/command/override") 
        self.client.loop_start()

    def get_new_waypoint(self):
        self.target_lat = 20.2960 + random.uniform(-0.012, 0.012)
        self.target_lon = 85.8240 + random.uniform(-0.012, 0.012)

    def on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode())
        
        if msg.topic == "lanex/charger/master/dispatch" and payload.get("vehicle_id") == self.vid:
            if payload["action"] == "pre_book_confirmed":
                self.pre_booked_charger = payload["assigned_charger"]
                self.deadline = payload["deadline"] 
                self.status = "working" 
            elif payload["action"] == "proceed_to_charge":
                self.status = "driving"
                self.charger = payload["assigned_charger"]
            elif payload["action"] == "standby_in_queue":
                self.status = "queued"
            elif payload["action"] == "abort_and_yield":
                self.status = "working"
                self.charger = None

        elif msg.topic == "lanex/command/override" and payload.get("target") == self.vid:
            self.status = payload["state"].lower()
            if self.status == "maintenance":
                self.charger = None
                self.pre_booked_charger = None

    def run(self):
        while True:
            is_real_hardware = (self.vid == "Cargo_A")

            real_power = None 
            real_discharge = None
            
            # --- HYBRID TWIN HEARTBEAT LOGIC ---
            if is_real_hardware:
                # If it's been more than 15 seconds since the last packet (or it never connected)
                if time.time() - cargo_a_real_telemetry["last_seen"] > 15:
                    if self.status != "maintenance":
                        print(f"[EDGE-LINK] ALERT: {self.vid} Heartbeat lost. Forcing asset offline.")
                        # If the car was charging when unplugged, release the charger
                        if self.charger:
                            self.client.publish("lanex/charger/slave/finished", json.dumps({"charger_id": self.charger}))
                        self.status = "maintenance"
                        self.charger = None
                        self.pre_booked_charger = None
                else:
                    # We have a live connection!
                    if self.status == "maintenance":
                        print(f"[EDGE-LINK] SUCCESS: {self.vid} connected. Restoring asset online.")
                        self.status = "working"
                        
                    self.soc = cargo_a_real_telemetry["soc"]
                    real_power = cargo_a_real_telemetry["power"]
                    real_discharge = cargo_a_real_telemetry["discharge_time"]

            # Countdown Timer for Reservations
            if self.deadline is not None:
                self.deadline -= 1
                if self.deadline <= 0:
                    self.pre_booked_charger = None 
                    self.deadline = None
            
            if self.status == "maintenance":
                pass

            elif self.status in ["working", "idle", "queued"]:
                # Only simulate battery drain if it's NOT the physical hardware
                if not is_real_hardware:
                    if self.status == "working":
                        if random.random() < 0.03: 
                            self.status = "idle"
                            self.idle_timer = random.randint(5, 12)
                    elif self.status == "idle":
                        self.idle_timer -= 1
                        if self.idle_timer <= 0:
                            self.status = "working"

                    drain_factor = 0.1 if self.status == "idle" else (0.5 + (self.payload_kg / 2000.0))
                    self.soc -= drain_factor
                
                # GPS Physics (Simulated for the map UI)
                if self.status == "working":
                    lat_dir = self.target_lat - self.lat
                    lon_dir = self.target_lon - self.lon
                    distance = (lat_dir**2 + lon_dir**2)**0.5
                    
                    if distance < 0.0005:
                        self.get_new_waypoint() 
                    else:
                        self.lat += (lat_dir / distance) * 0.0003
                        self.lon += (lon_dir / distance) * 0.0003
                
                # Intelligent Dispatching
                if self.soc <= 40 and not self.pre_booked_charger and self.status != "queued":
                    self.status = "pre_booking"
                    req = {"vehicle_id": self.vid, "soc": self.soc, "priority": self.priority, "type": "pre_book"}
                    self.client.publish("lanex/charger/master/request", json.dumps(req))

                if self.soc <= 20 and self.status in ["working", "idle"]:
                    self.status = "requesting"
                    req = {"vehicle_id": self.vid, "soc": self.soc, "priority": self.priority, "type": "immediate"}
                    self.client.publish("lanex/charger/master/request", json.dumps(req))
                    
            elif self.status == "driving":
                time.sleep(2)
                self.status = "charging"
                
            elif self.status == "charging":
                if not is_real_hardware:
                    self.soc += random.uniform(3.0, 5.0)
                
                if self.soc >= 80:
                    self.charge_cycles += 1 
                    self.client.publish("lanex/charger/slave/finished", json.dumps({"charger_id": self.charger}))
                    self.status = "working"
                    self.charger = None
                    self.pre_booked_charger = None
                    self.deadline = None
                    self.get_new_waypoint() 

            self.soc = max(0, min(100, self.soc))

            telemetry = {
                "vehicle_id": self.vid, "soc": round(self.soc, 1), "status": self.status,
                "payload": self.payload_kg, "pre_booked": self.pre_booked_charger, "deadline": self.deadline,
                "gps": {"lat": round(self.lat, 5), "lon": round(self.lon, 5)},
                "meta": {
                    "cycles": self.charge_cycles, "health": round(self.health_score, 1),
                    "last_maint": self.last_maint, "next_maint": self.next_maint,
                    "real_power": real_power,
                    "discharge_time": real_discharge
                }
            }
            self.client.publish("lanex/fleet/telemetry", json.dumps(telemetry))
            time.sleep(1)

# --- Main Execution ---
print("Starting LaneX Swarm with Edge-Link...")
udp_thread = threading.Thread(target=udp_listener, daemon=True)
udp_thread.start()

vehicles = [
    FleetVehicle("Cargo_A", "standard"), FleetVehicle("Cargo_B", "standard"),
    FleetVehicle("Cargo_C", "standard"), FleetVehicle("Cargo_D", "standard"),
    FleetVehicle("Cargo_E", "standard"), FleetVehicle("Cargo_F", "standard"),
    FleetVehicle("Logistics_G", "standard"), FleetVehicle("Logistics_H", "standard"),
    FleetVehicle("Logistics_I", "standard"), FleetVehicle("Logistics_J", "standard"),
    FleetVehicle("Ambulance_01", "high"), FleetVehicle("Ambulance_02", "high")
]

for v in vehicles:
    threading.Thread(target=v.run, daemon=True).start()

try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    print("Swarm offline.")