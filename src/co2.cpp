#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MHZ19.h>
#include <HardwareSerial.h>
#include <time.h> 
#include "aws_root_ca.h"
#include "aws_cert.h"
#include "aws_private_key.h"

const char* ssid = "Langat";
const char* password = "luwa2131";
const char* mqtt_server = "a7itckl5w2jb3-ats.iot.us-east-1.amazonaws.com";

WiFiClientSecure net;
PubSubClient client(net);

#define RX_PIN 16
#define TX_PIN 17
MHZ19 myMHZ19;
HardwareSerial mySerial(2);

void setupAWSClient() {
  net.setCACert(AWS_ROOT_CA);         // Root CA
  net.setCertificate(AWS_CERT); // Device certificate
  net.setPrivateKey(AWS_PRIVATE_KEY);  // Device private key
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  setupAWSClient();
  client.setServer(mqtt_server, 8883);
}

void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for time sync");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Time synced");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      delay(5000);
    }
  }
}


void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  myMHZ19.begin(mySerial);

  setup_wifi();
  syncTime();
}

unsigned long lastSendTime = 0;
const unsigned long interval = 30000;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  // keep connection alive

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= interval) {
    lastSendTime = currentMillis;

    int co2 = myMHZ19.getCO2();
    time_t now = time(nullptr);  // current Unix time

    Serial.print("Sending CO2: ");
    Serial.println(co2);

    String payload = "{";
    payload += "\"deviceId\":\"esp32-001\",";
    payload += "\"sensorType\":\"co2\",";
    payload += "\"timestamp\":" + String((uint64_t)now * 1000) + ",";
    payload += "\"co2\":" + String(co2);
    payload += "}";

    client.publish("sensorData", payload.c_str());
  }
}