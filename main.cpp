#include <Wire.h>                      // Includes the Wire library for I2C communication.
#include <Adafruit_I2CDevice.h>        // Includes the Adafruit I2C Device library to simplify I2C device interactions.
#include "ClosedCube_TCA9548A.h"       // Includes a library for the ClosedCube TCA9548A, an I2C multiplexer.
#include "M5Unified.h"                 // Includes the M5Unified library for using M5Stack specific features.
#include "SPIFFS.h"                    // Includes the SPIFFS (SPI Flash File System) library for handling file operations on SPI flash.
#include "SD.h"                        // Includes the library for using SD cards, enabling reading from and writing to SD cards.
#include "M5_ENV.h"                    // Includes a library specific to the M5Stack ENV unit, for environmental sensing.
#include <Unit_Sonic.h>                // Includes a library for the Ultrasonic Unit, likely for measuring distance.
#include <FastLED.h>                   // Includes the FastLED library, a popular choice for controlling addressable RGB LEDs.
#include <M5_DLight.h>                 // Includes a library for the M5Stack Digital Light sensor, for light sensing applications.
#include <WiFi.h>                      // Includes the WiFi library to enable WiFi functionality in your project.
#include <PubSubClient.h>              // Includes the PubSubClient library, commonly used for MQTT (Message Queuing Telemetry Transport) communication.
#include <ArduinoJson.h>               // Includes the ArduinoJson library, a JSON handling library for Arduino, useful for parsing or generating JSON data.

// Sensor and LED Definitions
#define TCA9548A_ADDRESS 0x70       // Defines the I2C address of the TCA9548A I2C multiplexer.
#define NUM_LEDS 60                 // Defines the number of LEDs in your LED strip.
#define DATA_PIN 14                 // Defines the data pin for the LED strip.
#define MOISTURE_SENSOR_PIN 36      // Defines the pin connected to a moisture sensor.
#define PUMP_PIN 26                 // Defines the pin controlling a pump, likely for water or nutrient delivery.
#define NCIR_ADDRESS 0x5A           // Defines the I2C address for an Infrared Thermometer, presumably the NCIR.


// WiFi and MQTT Configuration
const char* ssid = "";   // Your WiFi SSID
const char* password = ""; // Your WiFi password
const char* mqtt_server = "162.127.0.87"; // MQTT broker IP
const int mqtt_port = 1883; // MQTT broker port
const char* mqtt_topic = "/sensor/auto";

// Global Variables
ClosedCube::Wired::TCA9548A tca9548a;
SHT3X sht30;
SONIC_I2C sonic;
CRGB leds[NUM_LEDS];
M5_DLight dlight;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int LIGHT_THRESHOLD = 3500;
int currentBrightness = 50;
int moistureLevel;
bool pumpStatus = false;
float ncirTemperature = 0.0;

void setup() {
    M5.begin();
    Wire.begin();

    // Initialize the TCA9548A I2C multiplexer
    tca9548a.address(TCA9548A_ADDRESS);

    // Initialize the SHT30 sensor (ENV III)
    tca9548a.selectChannel(0);
    sht30.init();

    // Initialize the SONIC sensor
    tca9548a.selectChannel(1);
    sonic.begin();

    // Initialize the DLight sensor
    tca9548a.selectChannel(3);
    dlight.begin();
    dlight.setMode(CONTINUOUSLY_H_RESOLUTION_MODE);

    // Initialize the LED strip
    FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(currentBrightness);

    // Initialize the moisture sensor and pump
    pinMode(MOISTURE_SENSOR_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);

    // Setup Serial Communication
    Serial.begin(115200);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Connect to MQTT Broker
    mqttClient.setServer(mqtt_server, mqtt_port);
    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT...");
        if (mqttClient.connect("ESP32Client")) {
            Serial.println("Connected to MQTT");
        } else {
            delay(5000);
        }
    }
}

void loop() {
    // Reconnect to MQTT
    // Reconnect to MQTT if connection is lost
    if (!mqttClient.connected()) {
        while (!mqttClient.connected()) {
            Serial.println("Reconnecting to MQTT...");
            if (mqttClient.connect("ESP32Client")) {
                Serial.println("Reconnected to MQTT");
            } else {
                delay(5000);
            }
        }
    }
    mqttClient.loop();

    // Sensor Reading Variables
    float temperature = 0.0, humidity = 0.0, distance = 0.0, dlightLux = 0.0;

    // Read from SHT30 sensor
    tca9548a.selectChannel(0);
    if (sht30.get() == 0) {
        temperature = sht30.cTemp;
        humidity = sht30.humidity;
    } else {
        Serial.println("Failed to read from SHT30 sensor");
    }

    // Read from SONIC sensor
    tca9548a.selectChannel(1);
    distance = sonic.getDistance();
    if (distance == -1) {
        Serial.println("Failed to read from SONIC sensor");
    }

    // Read from DLight sensor
    tca9548a.selectChannel(3);
    dlightLux = dlight.getLUX();
    if (dlightLux == -1) {
        Serial.println("Failed to read from DLight sensor");
    }

    // Read from NCIR sensor
    tca9548a.selectChannel(2); 
    Wire.beginTransmission(NCIR_ADDRESS);
    Wire.write(0x07);
    Wire.endTransmission(false);
    Wire.requestFrom(NCIR_ADDRESS, 2);
    if(Wire.available() == 2) {
        uint16_t result = Wire.read();
        result |= Wire.read() << 8;
        ncirTemperature = result * 0.02 - 273.15;
    } else {
        Serial.println("Failed to read from NCIR sensor");
    }

    // Determine LED behavior based on DLight sensor
    if (dlightLux == 0) {
        // Dark environment: Turn off all LEDs
        fill_solid(leds, NUM_LEDS, CRGB::Black);
    } else if (dlightLux > 0 && dlightLux <= 20) {
        // Low room light: Moderate brightness, suitable for plant growth
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = (i % 2 == 0) ? CRGB::Red : CRGB::Blue;
        }
        FastLED.setBrightness(128); // Half brightness
    } else if (dlightLux >= 50) {
        // High room light: Lower brightness or off, as the room light might be enough
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = (i % 2 == 0) ? CRGB::Red : CRGB::Blue;
        }
        FastLED.setBrightness(64); // Lower brightness
    }
    FastLED.show();

    moistureLevel = analogRead(MOISTURE_SENSOR_PIN);

    // Automated pump control based on moisture level
    if (moistureLevel >= 2119 && moistureLevel <= 2180) { // Dry range
        pumpStatus = true;
        digitalWrite(PUMP_PIN, HIGH); // Turn on the pump
    } else if (moistureLevel >= 1600 && moistureLevel <= 1960) { // Wet range
        pumpStatus = false;
        digitalWrite(PUMP_PIN, LOW); // Turn off the pump
    }
    // If the moisture level is not in the dry or wet range, no action is taken on the pump


    // Update LCD Display
    M5.Lcd.clear();
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.printf("TEMP: %.2f C\n", temperature);
    M5.Lcd.setCursor(10, 30);
    M5.Lcd.printf("HUMI: %.2f%%\n", humidity);
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.printf("DIST: %.2f mm\n", distance);
    M5.Lcd.setCursor(10, 70);
    M5.Lcd.printf("DLIGHT: %.2f Lux\n", dlightLux);
    M5.Lcd.setCursor(10, 90);
    M5.Lcd.printf("Moisture: %d\n", moistureLevel);
    M5.Lcd.setCursor(10, 110);
    M5.Lcd.printf("Pump: %s\n", pumpStatus ? "ON" : "OFF");
    M5.Lcd.setCursor(10, 130); 
    M5.Lcd.printf("NCIR Temp: %.2f C\n", ncirTemperature);

    // Prepare the JSON string with sensor data
    String jsonData = "{";
    jsonData += "\"Temperature\":" + String(temperature, 2) + ",";
    jsonData += "\"Humidity\":" + String(humidity, 2) + ",";
    jsonData += "\"Distance\":" + String(distance, 2) + ",";
    jsonData += "\"Lux\":" + String(dlightLux, 2) + ",";
    jsonData += "\"Moisture\":" + String(moistureLevel) + ",";
    jsonData += "\"Pump Status\":\"" + String(pumpStatus ? "ON" : "OFF") + "\",";
    jsonData += "\"NCIRTemp\":" + String(ncirTemperature, 2);
    jsonData += "}";

    // Convert JSON string to char array
    char jsonOutput[512]; // Adjust the size based on your JSON data
    jsonData.toCharArray(jsonOutput, sizeof(jsonOutput));

    // Publish the JSON data to MQTT
    if (!mqttClient.publish(mqtt_topic, jsonOutput)) {
        Serial.println("Failed to publish MQTT message");
    }

    delay(1000);
}
