#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Hash.h>
#include "secrets.h"  // WiFi credentials + secret key — see secrets.h.example

Adafruit_INA219 ina219;
WiFiUDP udp;

// -------- Server --------
const char* serverIP = "192.168.137.126";
const int serverPort = 5005;

// -------- 2S Battery Config --------
const float minVoltage = 6.4;
const float maxVoltage = 8.4;
const float batteryCapacity_Ah = 2.2;

String generateSignature(String data) {
  String toHash = data + secretKey;
  return sha1(toHash);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(D2, D1);
  if (!ina219.begin()) {
    Serial.println("INA219 not found");
    while (1);
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi Connected");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(serverPort);
}

void loop() {
  float voltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA() / 1000.0;
  float power = ina219.getPower_mW() / 1000.0;

  // ---- SOC Calculation (2S) ----
  float soc = (voltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;
  if (soc > 100) soc = 100;
  if (soc < 0) soc = 0;

  // ---- Discharge Time Calculation ----
  float remainingAh = (soc / 100.0) * batteryCapacity_Ah;
  float dischargeTime = 0;
  if (current > 0.01) {
    dischargeTime = remainingAh / current;
  }

  // ---- Build JSON ----
  String payload = "{";
  payload += "\"soc\":" + String(soc, 1) + ",";
  payload += "\"power\":" + String(power, 2) + ",";
  payload += "\"discharge_time\":" + String(dischargeTime, 2);
  payload += "}";

  // ---- Signature ----
  String signature = generateSignature(payload);
  payload.remove(payload.length() - 1);
  payload += ",\"sig\":\"" + signature + "\"}";

  // ---- Send UDP ----
  udp.beginPacket(serverIP, serverPort);
  udp.print(payload);
  udp.endPacket();

  Serial.println("Packet Sent:");
  Serial.println(payload);

  delay(5000);
}
