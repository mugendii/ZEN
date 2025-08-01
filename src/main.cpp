#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MHZ19.h>
#include <HardwareSerial.h>
#include <time.h>
#include <max6675.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "aws_root_ca.h"
#include "aws_cert.h"
#include "aws_private_key.h"

// Wi-Fi and MQTT config
const char* ssid = "Langat";
const char* password = "luwa2131";
const char* mqtt_server = "a7itckl5w2jb3-ats.iot.us-east-1.amazonaws.com";

WiFiClientSecure net;
PubSubClient client(net);

// MH-Z19 (CO2 sensor)
#define RX_PIN 16
#define TX_PIN 17
MHZ19 myMHZ19;
HardwareSerial mySerial(2);

// MAX6675 (Thermocouple)
#define thermoCLK 18  // SCK
#define thermoCS 5    // CS
#define thermoDO 19   // SO (MISO)
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Pressure sensor (Analog)
#define pressurePin 34

// FreeRTOS handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t publishTaskHandle = NULL;

// Queue to pass sensor data between tasks
QueueHandle_t sensorDataQueue;

// Mutex for shared resources
SemaphoreHandle_t mqttMutex;

// Structure to hold sensor data
struct SensorData {
  int co2;
  float temperature;
  float pressure;
  uint64_t timestamp;
};

void setupAWSClient() {
  net.setCACert(AWS_ROOT_CA);
  net.setCertificate(AWS_CERT);
  net.setPrivateKey(AWS_PRIVATE_KEY);
}

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
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
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("Time synced");
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5 seconds");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// Task 1: Read sensors every 5 seconds
void sensorTask(void *parameter) {
  SensorData data;
  
  while (true) {
    // Read CO2
    data.co2 = myMHZ19.getCO2();
    
    // Read temperature
    data.temperature = thermocouple.readCelsius();
    
    // Read pressure
    int adcValue = analogRead(pressurePin);
    float voltage = adcValue * (4.5 / 1023.0);
    data.pressure =(voltage - 0.5) * (3.0 / 4.0);
    
    // Get timestamp
    data.timestamp = (uint64_t)time(nullptr) * 1000;
    
    // Send data to queue (non-blocking)
    if (xQueueSend(sensorDataQueue, &data, 0) != pdTRUE) {
      Serial.println("Failed to send sensor data to queue");
    }
    
    Serial.printf("Sensor readings - CO2: %d ppm, Temp: %.2f°C, Pressure: %.2f\n", 
                  data.co2, data.temperature, data.pressure);
    
    // Wait 5 seconds before next reading
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Task 2: Handle MQTT connection and keep alive
void mqttTask(void *parameter) {
  while (true) {
    // Take mutex to access MQTT client
    if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (!client.connected()) {
        reconnectMQTT();
      }
      client.loop();
      xSemaphoreGive(mqttMutex);
    }
    
    // Check every 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task 3: Publish sensor data every 30 seconds
void publishTask(void *parameter) {
  SensorData data;
  SensorData latestData = {0, 0.0, 0.0, 0};
  bool hasNewData = false;
  
  while (true) {
    // Check for new sensor data (non-blocking)
    while (xQueueReceive(sensorDataQueue, &data, 0) == pdTRUE) {
      latestData = data;
      hasNewData = true;
    }
    
    // Publish every 30 seconds if we have data
    if (hasNewData) {
      // Take mutex to access MQTT client
      if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
        if (client.connected()) {
          String payload = "{";
          payload += "\"deviceId\":\"esp32-001\",";
          payload += "\"timestamp\":" + String(latestData.timestamp) + ",";
          payload += "\"co2\":" + String(latestData.co2) + ",";
          payload += "\"temperature\":" + String(latestData.temperature) + ",";
          payload += "\"pressure\":" + String(latestData.pressure);
          payload += "}";
          
          Serial.println("Publishing payload:");
          Serial.println(payload);
          
          if (client.publish("sensorData", payload.c_str())) {
            Serial.println("Data published successfully");
          } else {
            Serial.println("Failed to publish data");
          }
        }
        
        xSemaphoreGive(mqttMutex);
        hasNewData = false;
      }
    }
    
    // Wait 30 seconds before next publish attempt
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

// Task 4: Monitor system health and free memory
void systemMonitorTask(void *parameter) {
  while (true) {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("WiFi status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("MQTT status: %s\n", client.connected() ? "Connected" : "Disconnected");
    Serial.println("---");
    
    // Report every 60 seconds
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize serial for MH-Z19
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  myMHZ19.begin(mySerial);
  
  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  // Connect to WiFi and sync time
  setup_wifi();
  syncTime();
  
  // Create queue for sensor data (hold up to 5 readings)
  sensorDataQueue = xQueueCreate(5, sizeof(SensorData));
  if (sensorDataQueue == NULL) {
    Serial.println("Failed to create sensor data queue");
    return;
  }
  
  // Create mutex for MQTT client
  mqttMutex = xSemaphoreCreateMutex();
  if (mqttMutex == NULL) {
    Serial.println("Failed to create MQTT mutex");
    return;
  }
  
  // Create tasks
  xTaskCreatePinnedToCore(
    sensorTask,           // Task function
    "SensorTask",         // Task name
    4096,                 // Stack size (bytes)
    NULL,                 // Parameters
    2,                    // Priority (0-25, higher number = higher priority)
    &sensorTaskHandle,    // Task handle
    1                     // Core (0 or 1)
  );
  
  xTaskCreatePinnedToCore(
    mqttTask,
    "MQTTTask",
    4096,
    NULL,
    3,                    // Higher priority for connectivity
    &mqttTaskHandle,
    0                     // Core 0
  );
  
  xTaskCreatePinnedToCore(
    publishTask,
    "PublishTask",
    4096,
    NULL,
    2,
    &publishTaskHandle,
    0                     // Core 0
  );
  
  xTaskCreatePinnedToCore(
    systemMonitorTask,
    "SystemMonitor",
    2048,
    NULL,
    1,                    // Lowest priority
    NULL,
    1                     // Core 1
  );
  
  Serial.println("FreeRTOS tasks created successfully");
}

void loop() {
  // Empty - everything is handled by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}