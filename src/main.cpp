//#include <WiFi.h>
//#include <WiFiClientSecure.h>
//#include <PubSubClient.h>
//#include <MHZ19.h>
//#include <HardwareSerial.h>
//#include <time.h>
//#include <max6675.h>
//#include "aws_root_ca.h"
//#include "aws_cert.h"
//#include "aws_private_key.h"
//
//// Wi-Fi and MQTT config
//const char* ssid = "Langat";
//const char* password = "luwa2131";
//const char* mqtt_server = "a7itckl5w2jb3-ats.iot.us-east-1.amazonaws.com";
//
//WiFiClientSecure net;
//PubSubClient client(net);
//
//// MH-Z19 (CO2 sensor)
//#define RX_PIN 16
//#define TX_PIN 17
//MHZ19 myMHZ19;
//HardwareSerial mySerial(2);
//
//// MAX6675 (Thermocouple)
//#define thermoCLK 18  // SCK
//#define thermoCS 5    // CS
//#define thermoDO 19   // SO (MISO)
//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
//
//// Pressure sensor (Analog)
//#define pressurePin 34
//
//void setupAWSClient() {
//  net.setCACert(AWS_ROOT_CA);
//  net.setCertificate(AWS_CERT);
//  net.setPrivateKey(AWS_PRIVATE_KEY);
//}
//
//void setup_wifi() {
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("WiFi connected");
//  setupAWSClient();
//  client.setServer(mqtt_server, 8883);
//}
//
//void syncTime() {
//  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
//  Serial.print("Waiting for time sync");
//  while (time(nullptr) < 100000) {
//    Serial.print(".");
//    delay(500);
//  }
//  Serial.println("Time synced");
//}
//
//void reconnect() {
//  while (!client.connected()) {
//    Serial.print("Attempting MQTT connection...");
//    if (client.connect("ESP32Client")) {
//      Serial.println("connected");
//    } else {
//      delay(5000);
//    }
//  }
//}
//
//void setup() {
//  Serial.begin(115200);
//  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
//  myMHZ19.begin(mySerial);
//
//  analogReadResolution(12);
//  analogSetAttenuation(ADC_11db);
//
//  setup_wifi();
//  syncTime();
//  delay(500);
//}
//
//unsigned long lastSendTime = 0;
//const unsigned long interval = 10000;
//
//void loop() {
//  if (!client.connected()) {
//    reconnect();
//  }
//  client.loop();
//
//  unsigned long currentMillis = millis();
//  if (currentMillis - lastSendTime >= interval) {
//    lastSendTime = currentMillis;
//    time_t now = time(nullptr);
//
//    int co2 = myMHZ19.getCO2();
//    float tempC = thermocouple.readCelsius();
//    int adcValue = analogRead(pressurePin);
//    float voltage = adcValue * (3.3 / 4095.0);
//    float pressure = (voltage - 0.2) * (700.0 / 2.9);
//
//    String payload = "{";
//    payload += "\"deviceId\":\"esp32-001\",";
//    payload += "\"timestamp\":" + String((uint64_t)now * 1000) + ",";
//    payload += "\"co2\":" + String(co2) + ",";
//    payload += "\"temperature\":" + String(tempC) + ",";
//    payload += "\"pressure\":" + String(48.0);
//    payload += "}";
//
//    Serial.println("Sending payload:");
//    Serial.println(payload);
//    client.publish("sensorData", payload.c_str());
//  }
//}










#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MHZ19.h>
#include <HardwareSerial.h>
#include <time.h>
#include <max6675.h>
#include "aws_root_ca.h"
#include "aws_cert.h"
#include "aws_private_key.h"

// WiFi and MQTT config
const char* ssid = "Langat";
const char* password = "luwa2131";
const char* mqtt_server = "a7itckl5w2jb3-ats.iot.us-east-1.amazonaws.com";

WiFiClientSecure net;
PubSubClient client(net);

bool wifiReady = false;

// MH-Z19 (CO2 sensor)
#define RX_PIN 16
#define TX_PIN 17
MHZ19 myMHZ19;
HardwareSerial mySerial(2);

// MAX6675 (Thermocouple)
#define thermoCLK 18
#define thermoCS 5
#define thermoDO 19
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Pressure sensor
#define pressurePin 34

void setupAWSClient() {
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);
}

void setup_wifi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");

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
  Serial.println("\nTime synced");
  wifiReady = true;
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttTask(void* parameter) {
  while (!wifiReady) {
    Serial.println("MQTT task waiting for Wi-Fi and time sync...");
    delay(1000);
  }

  while (true) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    delay(10);
  }
}

void publishTask(void* parameter) {
  const unsigned long interval = 10000;
  unsigned long lastSendTime = 0;

  while (!wifiReady) {
    Serial.println("Publisher task waiting for Wi-Fi and time sync...");
    delay(1000);
  }

  while (true) {
    unsigned long nowMillis = millis();
    if (nowMillis - lastSendTime >= interval) {
      lastSendTime = nowMillis;

      time_t now = time(nullptr);
      int co2 = myMHZ19.getCO2();
      float tempC = thermocouple.readCelsius();
      int adcValue = analogRead(pressurePin);
      float voltage = adcValue * (4.5/ 1023.0); // Adjusted for 12-bit ADC
      float pressure =(voltage - 0.5) * (3.0 / 4.0);

      String payload = "{";
      payload += "\"deviceId\":\"esp32-001\",";
      payload += "\"timestamp\":" + String((uint64_t)now * 1000) + ",";
      payload += "\"co2\":" + String(co2) + ",";
      payload += "\"temperature\":" + String(tempC) + ",";
      payload += "\"pressure\":" + String(pressure);
      payload += "}";

      Serial.println("Publishing payload:");
      Serial.println(payload);

      client.publish("sensorData", payload.c_str());
    }

    delay(100);
  }
}

void setup() {
  Serial.begin(115200);

  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  myMHZ19.begin(mySerial);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setup_wifi();
  syncTime();

  xTaskCreatePinnedToCore(
    mqttTask,      // Task function
    "MQTT Task",   // Task name
    4096,          // Stack size
    NULL,          // Params
    1,             // Priority
    NULL,          // Task handle
    0              // Run on core 0
  );

  xTaskCreatePinnedToCore(
    publishTask,   // Task function
    "Publish Task",// Task name
    4096,          // Stack size
    NULL,          // Params
    1,             // Priority
    NULL,          // Task handle
    1              // Run on core 1
  );
}

void loop() {
  // Nothing here — all handled by FreeRTOS tasks
}