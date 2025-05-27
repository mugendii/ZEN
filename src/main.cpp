#include <Arduino.h>

#include <max6675.h>

// MAX6675 pins (VSPI)
#define thermoCLK 18  // SCK
#define thermoCS 5    // CS
#define thermoDO 19   // SO (MISO)

// Pressure sensor pin
#define pressurePin 34  // ADC1_CHANNEL_6

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  Serial.begin(115200);
  Serial.println("MAX6675 Thermocouple and Pressure Sensor Test");
  
  // Configure ADC
  analogReadResolution(12);  // Set ADC to 12-bit (0-4095)
  analogSetAttenuation(ADC_11db);  // Full 3.3V range
  
  delay(500);  // Wait for MAX6675 and sensor to stabilize
}

void loop() {
  // Read temperature
  float tempC = thermocouple.readCelsius();
  float tempF = thermocouple.readFahrenheit();
  
  // Read pressure
  int adcValue = analogRead(pressurePin);
  float voltage = adcValue * (3.3 / 4095.0);  // Convert to voltage
  float pressure = (voltage - 0.2) * (700.0 / 2.9);  // Convert to kPa (MPX5700AP)
  
  // Output results
  if (isnan(tempC)) {
    Serial.println("Thermocouple disconnected or error!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" °C, ");
    Serial.print(tempF);
    Serial.println(" °F");
  }
  
  if (pressure < 0 || pressure > 700) {
    Serial.println("Pressure sensor error or out of range!");
  } else {
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" kPa");
  }
  
  delay(1000);  // Wait for MAX6675 conversion (min 220ms)
}