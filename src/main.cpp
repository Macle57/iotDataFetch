#include "select.h"
#ifdef MAIN
#include <Arduino.h>
#include <ArduinoJson.h>

// Enable/disable sensors here
// #define ENABLE_BME280      // BME280 temperature, pressure, humidity, altitude
#define ENABLE_DHT11       // DHT11 temperature, humidity, heat index
#define ENABLE_SOIL_MOISTURE  // Soil moisture sensor
#define ENABLE_SOIL_TEMP      // DS18B20 soil temperature sensor

// Sensor-specific includes
#ifdef ENABLE_BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif

#ifdef ENABLE_DHT11
#include <DHT.h>
#endif

#ifdef ENABLE_SOIL_TEMP
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

// Pin definitions
#ifdef ENABLE_DHT11
#define DHTPIN 18
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef ENABLE_SOIL_MOISTURE
#define SOIL_MOISTURE_PIN 27
#endif

#ifdef ENABLE_SOIL_TEMP
#define ONE_WIRE_BUS 26
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature soilTempSensor(&oneWire);
#endif

#ifdef ENABLE_BME280
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
#endif

// Function declarations
void initializeSensors();
void readSensorData(JsonDocument& doc);

#ifdef ENABLE_BME280
void readBME280(JsonDocument& doc);
#endif

#ifdef ENABLE_DHT11
void readDHT11(JsonDocument& doc);
#endif

#ifdef ENABLE_SOIL_TEMP
void readSoilTemperature(JsonDocument& doc);
#endif

#ifdef ENABLE_SOIL_MOISTURE
void readSoilMoisture(JsonDocument& doc);
#endif

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  
  Serial.println(F("Multi-Sensor JSON Reader"));
  Serial.println(F("========================"));
  
  initializeSensors();
  
  Serial.println();
  delay(2000);
}

void loop() {
  JsonDocument doc;
  doc["timestamp"] = millis();
  
  readSensorData(doc);
    // The doc now contains the data, call your function here to process it
    // yourFunction(doc);
  
  
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(2000);
}

// Initialize all enabled sensors
void initializeSensors() {
  #ifdef ENABLE_BME280
  Wire.begin();
  delay(100);
  if (!bme.begin(0x76, &Wire)) {
    Serial.println(F("BME280 initialization failed!"));
  } else {
    Serial.println(F("BME280 initialized"));
  }
  #endif
  
  #ifdef ENABLE_DHT11
  dht.begin();
  Serial.println(F("DHT11 initialized"));
  #endif
  
  #ifdef ENABLE_SOIL_TEMP
  soilTempSensor.begin();
  Serial.println(F("DS18B20 soil temperature initialized"));
  #endif
  
  #ifdef ENABLE_SOIL_MOISTURE
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  Serial.println(F("Soil moisture sensor initialized"));
  #endif
}

// Read data from all enabled sensors
void readSensorData(JsonDocument& doc) {
  #ifdef ENABLE_BME280
  readBME280(doc);
  #endif
  
  #ifdef ENABLE_DHT11
  readDHT11(doc);
  #endif
  
  #ifdef ENABLE_SOIL_TEMP
  readSoilTemperature(doc);
  #endif
  
  #ifdef ENABLE_SOIL_MOISTURE
  readSoilMoisture(doc);
  #endif
}

#ifdef ENABLE_BME280
// Read BME280 sensor (temperature, pressure, humidity, altitude)
void readBME280(JsonDocument& doc) {
  JsonObject bme280 = doc["bme280"].to<JsonObject>();
  bme280["temperature"] = round(bme.readTemperature() * 100) / 100.0;
  bme280["pressure"] = round(bme.readPressure() / 100.0F * 100) / 100.0;
  bme280["humidity"] = round(bme.readHumidity() * 100) / 100.0;
  bme280["altitude"] = round(bme.readAltitude(SEALEVELPRESSURE_HPA) * 100) / 100.0;
}
#endif

#ifdef ENABLE_DHT11
// Read DHT11 sensor (temperature, humidity, heat index)
void readDHT11(JsonDocument& doc) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (!isnan(h) && !isnan(t)) {
    JsonObject dht11 = doc["dht11"].to<JsonObject>();
    dht11["temperature"] = round(t * 100) / 100.0;
    dht11["humidity"] = round(h * 100) / 100.0;
    dht11["heatIndex"] = round(dht.computeHeatIndex(t, h, false) * 100) / 100.0;
  } else {
    doc["dht11"] = "error";
  }
}
#endif

#ifdef ENABLE_SOIL_TEMP
// Read DS18B20 soil temperature sensor
void readSoilTemperature(JsonDocument& doc) {
  soilTempSensor.requestTemperatures();
  delay(50); // Give sensor time to read
  float soilTempC = soilTempSensor.getTempCByIndex(0);
  float soilTempF = soilTempSensor.getTempFByIndex(0);
  
  JsonObject soilTempData = doc["soilTemperature"].to<JsonObject>();
  soilTempData["celsius"] = round(soilTempC * 100) / 100.0;
  soilTempData["fahrenheit"] = round(soilTempF * 100) / 100.0;
}
#endif

#ifdef ENABLE_SOIL_MOISTURE
// Read analog soil moisture sensor
void readSoilMoisture(JsonDocument& doc) {
  int soilMoisture = analogRead(SOIL_MOISTURE_PIN);
  JsonObject soilData = doc["soilMoisture"].to<JsonObject>();
  soilData["raw"] = soilMoisture;
  soilData["percentage"] = map(soilMoisture, 0, 4095, 0, 100);
}
#endif

#endif

