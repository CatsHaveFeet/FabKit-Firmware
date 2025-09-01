#define FWver "1.0.6" // current Firmware version

#include <Preferences.h>
#include "nvs_flash.h"
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <cstring>

Preferences preferences;
String serialNumber = "";
String macAddress = "";
String boardType = "";

// Pin definitions
const int analogPin1 = 34; // D34
const int analogPin2 = 35; // D35

// WS2812 LED definitions
#include <FastLED.h>
#define NUM_LEDS 1
#define DATA_PIN 2   // D2 pin
CRGB leds[NUM_LEDS]; // Define the array of leds

// Data Structure for Sensor Data
typedef struct
{
  char identifier;    // Sensor identifier 'C' 'T'
  String sensoralias; // Sensor name "Curent" "Tensiune"
  uint32_t timestamp; // Milliseconds timestamp
  float values[5];    // Up to 5 float values! (ensure that in code)
  uint8_t valueCount; // Number of float values actually used
} SensorData;

// Global Variables
SensorData sensorDataArray;
uint32_t currentTime;       // current timestamp in SensorTask
float sensorVal[5] = {0.0}; // sensor values array (up to 5 values accepted) in SensorTask
SemaphoreHandle_t xMutex;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t commTaskHandle = NULL;
TaskHandle_t arduinoTaskHandle = NULL;

// Definitions for SENSORS

// Definitions for Fabkit Light module
#include "Adafruit_VEML7700.h"
Adafruit_VEML7700 veml = Adafruit_VEML7700();

// Definitions for Temperature module
#include <SPI.h>
#define RREF 430.0           // The value of the RTD's resistor, usually 430.0 for PT100
#define RNOMINAL 100.0       // The nominal resistance of the RTD, 100.0 for PT100
#define MAX31865_CS_PIN 5    // Chip Select (CS) pin for MAX31865
#define MAX31865_RDY_PIN 4   // RDY pin connected to digital pin 4
#define MAX31865_MISO_PIN 19 // MISO (Master In Slave Out)
#define MAX31865_SCK_PIN 18  // SCK (Serial Clock)
#define MAX31865_MOSI_PIN 23 // MOSI (Master Out Slave In)
float TempeLastTemperature = 0;
float TempeCurrentTemperature = 0;
unsigned long TempeLastSampleTime = 0;               // Time when the last real sample was taken
unsigned long TempeNextInterpolationTime = 0;        // Time for the next interpolated point
const unsigned long TempeInterpolationInterval = 10; // Interpolate every 10ms for 100Hz
bool TempeNewRealDataAvailable = false;

// Definitions for Pressure module
#define SDA_PIN 21 // Define the I2C pins for ESP32
#define SCL_PIN 22
#define SENSOR_ADDR 0x6D      // I2C address of the XGZP6847D sensor
#define REG_PRESSURE_MSB 0x06 // Register addresses
#define REG_PRESSURE_CSB 0x07
#define REG_PRESSURE_LSB 0x08
#define REG_CMD 0x30
#define K_VALUE 16.0 // K value for 0 to 500 kPa range

// Definitions for Distance module
#include "SparkFun_VL53L1X.h" // Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#define SHUTDOWN_PIN 32       // Optional interrupt and shutdown pins.
#define INTERRUPT_PIN 26
SFEVL53L1X distanceSensor;
// SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN); // Nu a mers
float VLsmoothingFactor = 0.2;            // Smoothing factor (alpha), adjust between 0 and 1
float VLsmoothedDistance = 0;             // Smoothed distance
float prevDistance = 0, currDistance = 0; // Distances for speed calculation
float prevSpeed = 0, currSpeed = 0;       // Speeds for acceleration calculation
float acceleration = 0;                   // Acceleration calculation
unsigned long VLlastSampleTime = 0;       // Time when last valid reading was taken
unsigned long VLcurrentTime = 0;

// Definitions for Current module
#include <Adafruit_ADS1X15.h> // Then create two instances for the ADS1015 :
Adafruit_ADS1015 ads1;        // ADS1015 instance #1
Adafruit_ADS1015 ads2;        // ADS1015 instance #2

// Function prototypes
void SensorTask(void *pvParameters);
void CommunicationTask(void *pvParameters);
void ArduinoTask(void *pvParameters);
void messageReceived(String &topic, String &payload);
void connect();
void SensorSemaphoreSend();
void deleteTasks(TaskHandle_t *commTaskHandle, TaskHandle_t *arduinoTaskHandle);
bool withinTolerance(float measured, float expected, float tolerance);
float readPressure();
float TempeReadTemperature();

//Wi-Fi credentials
String ssid = "";  
String password = "";

//Communication options
bool serialComm = false;
bool wifiComm = false;

//MQTT configuration
#define mqttServer "192.168.1.11"
WiFiClient espClient;
PubSubClient client(espClient);
char *freq;
String mqttBaseTopic;
String dataTopic;
String freqTopic;


#define MAX_MSG_LEN 64
char recTopic[64];
char recMessage[MAX_MSG_LEN];

void mqttCallback(char* topic, byte* message, unsigned int length) {
  // copy topic safely
  strncpy(recTopic, topic, sizeof(recTopic) - 1);
  recTopic[sizeof(recTopic) - 1] = '\0';

  // copy message safely
  int copyLen = min((int)length, MAX_MSG_LEN - 1);
  memcpy(recMessage, message, copyLen);
  recMessage[copyLen] = '\0'; // null terminate
}


void setup()
{
  // Serial ports and LED setup
  Serial.begin(460800);                                // For Communication to PC
                                                       // Serial2.begin(9600);  // Initialize Serial2 with straight RX and TX pins (16 and 17)
  Serial2.begin(9600, SERIAL_8N1, 17, 16);             // Initialize Serial2 with swapped RX and TX pins
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); // Initialize LED, GRB ordering is assumed
  FastLED.setBrightness(70);                           // Set brightness from 0 to 255
  leds[0] = CRGB::Red;                                 // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
  FastLED.show();                                      // Update LED

  // Check if "EraseNVS" is received for the first second after master boot
  delay(1000); // Wait for 1 second
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "EraseNVS")
    {
      leds[0] = CRGB::Purple; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
      FastLED.show();         // Update LED
      Serial.println("EraseNVS command received. Erasing NVS...");
      esp_err_t result = nvs_flash_erase(); // Erase all the NVS (Non-Volatile Storage) data
      if (result == ESP_OK)
      {
        Serial.println("NVS flash erased successfully.");
      }
      else
      {
        Serial.printf("Error erasing NVS flash: %d\n", result);
      }
      delay(2000); // Wait for 2 seconds
      Serial.println("Rebooting...");
      delay(2000);           // Wait for 2 seconds
      leds[0] = CRGB::Black; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
      FastLED.show();        // Update LED
      ESP.restart();         // Restart ESP32
    }
  }
  // Print firmware version
  leds[0] = CRGB::Pink; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
  FastLED.show();       // Update LED
  Serial.println();
  Serial.println();
  Serial.println("Hello FabKit !!");
  Serial.print("Firmware ver. ");
  Serial.println(FWver);

  // Set ADC resolution and attenuation
  analogReadResolution(12);       // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db); // Set attenuation to 11 dB for full-scale 3.3V readings
  delay(10);

  // Read the MAC address
  macAddress = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);

  // Read analog values from D34 and D35
  int adcValue1 = analogRead(analogPin1);
  int adcValue2 = analogRead(analogPin2);

  // Convert ADC readings to voltage
  float voltage1 = (adcValue1 * 3.3) / 4095.0; // 12-bit ADC resolution
  float voltage2 = (adcValue2 * 3.3) / 4095.0;

  Serial.print("ADC at D34, D35: ");
  Serial.print(voltage1, 3);
  Serial.print("V, ");
  Serial.print(voltage2, 3);
  Serial.println("V");

  // Identify the board type using ±0.3V tolerance
  float tolerance = 0.27; // ±0.27V tolerance (ESP32 ADC is not precise)

  // Expected voltages
  float fabLightVoltage = 3.3 / 2;                    // 1.65 V
  float fabTempVoltage = 0.0;                         // 0 V
  float fabPressureVoltage = 3.3;                     // 3.3 V
  float fabDistanceVoltage = 3.3 * 680 / (680 + 330); // ≈2.27 V
  float fabCurrentVoltage = 3.3 * 330 / (680 + 330);  // ≈1.03 V

  // Use a function to check if measured voltages are within ±0.27 tolerance
  if (withinTolerance(voltage1, fabLightVoltage, tolerance) && withinTolerance(voltage2, 0.0, tolerance))
  {
    boardType = "FabLight";
    sensorDataArray.identifier = 'L';
    sensorDataArray.sensoralias = "Lumina";
    sensorDataArray.valueCount = 1;
    mqttBaseTopic = "FabKit/FabLight";
  }
  else if (withinTolerance(voltage1, fabTempVoltage, tolerance) && withinTolerance(voltage2, 0.0, tolerance))
  {
    boardType = "FabTemp";
    sensorDataArray.identifier = 'T';
    sensorDataArray.sensoralias = "Temperatura";
    sensorDataArray.valueCount = 1;
    mqttBaseTopic = "FabKit/FabTemp";
  }
  else if (withinTolerance(voltage1, fabPressureVoltage, tolerance) && withinTolerance(voltage2, 0.0, tolerance))
  {
    boardType = "FabPressure";
    sensorDataArray.identifier = 'P';
    sensorDataArray.sensoralias = "Presiune";
    sensorDataArray.valueCount = 1;
    mqttBaseTopic = "FabKit/FabPress";
  }
  else if (withinTolerance(voltage1, fabDistanceVoltage, tolerance) && withinTolerance(voltage2, 0.0, tolerance))
  {
    boardType = "FabMove";
    sensorDataArray.identifier = 'D';
    sensorDataArray.sensoralias = "Distanta";
    sensorDataArray.valueCount = 3;
    mqttBaseTopic = "FabKit/FabMove";
  }
  else if (withinTolerance(voltage1, fabCurrentVoltage, tolerance) && withinTolerance(voltage2, 0.0, tolerance))
  {
    boardType = "FabCurrent";
    sensorDataArray.identifier = 'C';
    sensorDataArray.sensoralias = "Curent";
    sensorDataArray.valueCount = 2;
    mqttBaseTopic = "FabKit/FabCurrent";
  }
  else
  {
    boardType = "Unknown";
    Serial.println("ERROR. Board unknown, halting...");
    while (1)
      ; // Halt execution
  }

  
  dataTopic = mqttBaseTopic + "/Data";
  freqTopic = mqttBaseTopic + "/Freq";

  Serial.print("Board Type found: ");
  Serial.println(boardType);
  Serial.print("Board Identifier: ");
  Serial.println(sensorDataArray.identifier);

  // Now proceed to Preferences
  preferences.begin("my_app", false);

  //Skip setup for wifi only mode
  preferences.getBool("SerialComm", serialComm);
  preferences.getBool("WifiComm", wifiComm);

  // Read the serial number from Preferences
  serialNumber = preferences.getString("serialNumber", "");

  if (serialComm || (!serialComm && !wifiComm)) {
  
    // Check if the serial number is empty
    if (serialNumber.length() == 0)
    {
      Serial.println("Serial number not found in storage.");
      while (true) {
        Serial.println("Please enter the serial number:");

        // Wait for user input

        // Serial.setTimeout(4294967295); // Set the timeout to the maximum value (49days)
        while (Serial.available() == 0)
        {
          // Do nothing, just wait
        }

        // Read the serial number from the serial monitor
        serialNumber = Serial.readStringUntil('\n'); // LF, or change witk \r for CR (some terminals use CR)
        serialNumber.trim();                         // Remove any leading/trailing whitespace

        //Asks if input is correct
        Serial.print("You have entered ");
        Serial.print(serialNumber);
        Serial.println(". Is this correct? (y/n)");

        String response = "";

        while (true) {

          //Wait for user response
          while (Serial.available() == 0) {

          }

          //Process input
          response = Serial.readStringUntil('\n');
          response.trim();
          if (response != "N" && response != "n" && response != "Y" && response != "y") {
            Serial.println("Invalid response!");
          }
          else {
            break;
          }
        }

        //Check response
        if (response == "Y" || response == "y") {
          break;
        }
      }
      // Save the serial number to Preferences
      preferences.putString("serialNumber", serialNumber);
      Serial.print("Serial number ");
      Serial.print(serialNumber);
      Serial.println(" saved to storage.");
      // Serial.setTimeout(1000); // Set the timeout back to default
    }
    else
    {
      Serial.print("Serial number read from storage: ");
      Serial.println(serialNumber);

      // Save the serial number to Preferences
      preferences.putString("serialNumber", serialNumber);
      Serial.print("Serial number ");
      Serial.print(serialNumber);
      Serial.println(" saved to storage.");
    }

    //Checks for saved SSID
    ssid = preferences.getString("SSID", "");
    password = preferences.getString("Password", "");

      // Check if the SSID is empty
    if (ssid.length() == 0)
    {
      Serial.println("SSID not found in storage.");
      while (true) {
        Serial.println("Please enter the SSID:");

        // Wait for user input

        // Serial.setTimeout(4294967295); // Set the timeout to the maximum value (49days)
        while (Serial.available() == 0)
        {
          // Do nothing, just wait
        }

        // Read the serial number from the serial monitor
        ssid = Serial.readStringUntil('\n'); // LF, or change witk \r for CR (some terminals use CR)
        ssid.trim();                         // Remove any leading/trailing whitespace

        //Asks if input is correct
        Serial.print("You have entered ");
        Serial.print(ssid);
        Serial.println(". Is this correct? (y/n)");

        String response = "";

        while (true) {

          //Wait for user response
          while (Serial.available() == 0) {

          }

          //Process input
          response = Serial.readStringUntil('\n');
          response.trim();
          if (response != "N" && response != "n" && response != "Y" && response != "y") {
            Serial.println("Invalid response!");
          }
          else {
            break;
          }
        }

        //Check response
        if (response == "Y" || response == "y") {
          break;
        }
      }
      // Save the SSID to Preferences
      preferences.putString("SSID", ssid);
      Serial.print("SSID ");
      Serial.print(ssid);
      Serial.println(" saved to storage.");

      while (true) {
        //Ask for password
        Serial.println("Please enter network password:");

        // Wait for user input

        // Serial.setTimeout(4294967295); // Set the timeout to the maximum value (49days)
        while (Serial.available() == 0)
        {
          // Do nothing, just wait
        }

        password = Serial.readStringUntil('\n'); // LF, or change witk \r for CR (some terminals use CR)
        password.trim();                         // Remove any leading/trailing whitespace

        //Asks if input is correct
        Serial.print("You have entered ");
        Serial.print(password);
        Serial.println(". Is this correct? (y/n)");

        String response = "";

        while (true) {

          //Wait for user response
          while (Serial.available() == 0) {

          }

          //Process input
          response = Serial.readStringUntil('\n');
          response.trim();
          if (response != "N" && response != "n" && response != "Y" && response != "y") {
            Serial.println("Invalid response!");
          }
          else {
            break;
          }
        }

        //Check response
        if (response == "Y" || response == "y") {
          break;
        }
      }
      // Save the SSID to Preferences
      preferences.putString("Password", password);
      Serial.print("Password ");
      Serial.print(password);
      Serial.println(" saved to storage.");
      
    }
    else
    {
      Serial.print("SSID read from storage: ");
      Serial.println(ssid);
      Serial.print("Password read from storage: ");
      Serial.println(password);

      //Asks if user wants to change value
      
      Serial.println("Do you wish to change the SSID? (y/n)");

      String response = "";

      while (true) {

        //Wait for user response
        while (Serial.available() == 0) {

        }

        //Process input
        response = Serial.readStringUntil('\n');
        response.trim();
        if (response != "N" && response != "n" && response != "Y" && response != "y") {
          Serial.println("Invalid response!");
        }
        else {
          break;
        }
      }

      //Check response
      if (response == "Y" || response == "y") {
        while (true) {
          Serial.println("Please enter the SSID:");

          // Wait for user input

          // Serial.setTimeout(4294967295); // Set the timeout to the maximum value (49days)
          while (Serial.available() == 0)
          {
            // Do nothing, just wait
          }

          // Read the serial number from the serial monitor
          ssid = Serial.readStringUntil('\n'); // LF, or change witk \r for CR (some terminals use CR)
          ssid.trim();                         // Remove any leading/trailing whitespace

          //Asks if input is correct
          Serial.print("You have entered ");
          Serial.print(ssid);
          Serial.println(". Is this correct? (y/n)");

          String response = "";

          while (true) {

            //Wait for user response
            while (Serial.available() == 0) {

            }

            //Process input
            response = Serial.readStringUntil('\n');
            response.trim();
            if (response != "N" && response != "n" && response != "Y" && response != "y") {
              Serial.println("Invalid response!");
            }
            else {
              break;
            }
          }

          //Check response
          if (response == "Y" || response == "y") {
            break;
          }
        }
        // Save the SSID to Preferences
        preferences.putString("SSID", ssid);
        Serial.print("SSID ");
        Serial.print(ssid);
        Serial.println(" saved to storage.");

        while (true) {
          //Ask for password
          Serial.println("Please enter network password:");

          // Wait for user input

          // Serial.setTimeout(4294967295); // Set the timeout to the maximum value (49days)
          while (Serial.available() == 0)
          {
            // Do nothing, just wait
          }

          password = Serial.readStringUntil('\n'); // LF, or change witk \r for CR (some terminals use CR)
          password.trim();                         // Remove any leading/trailing whitespace

          //Asks if input is correct
          Serial.print("You have entered ");
          Serial.print(password);
          Serial.println(". Is this correct? (y/n)");

          String response = "";

          while (true) {

            //Wait for user response
            while (Serial.available() == 0) {

            }

            //Process input
            response = Serial.readStringUntil('\n');
            response.trim();
            if (response != "N" && response != "n" && response != "Y" && response != "y") {
              Serial.println("Invalid response!");
            }
            else {
              break;
            }
          }

          //Check response
          if (response == "Y" || response == "y") {
            break;
          }
        }
        // Save the serial number to Preferences
        preferences.putString("Password", password);
        Serial.print("Password ");
        Serial.print(password);
        Serial.println(" saved to storage.");
      }
    }
    
    //Asks user for communicaton channels
    Serial.println("Do you wish to use communication over serial, wifi or both? (S/W/B/N(skip))");
    String response = "";

    while (true) {

      //Wait for user response
      while (Serial.available() == 0) {

      }

      //Process input
      response = Serial.readStringUntil('\n');
      response.trim();
      if (response != "S" && response != "s" && response != "W" && response != "w" && response != "B" && response != "b" && response == "N" && response == "n") {
        Serial.println("Invalid response!");
      }
      else {
        break;
      }
    }

    //Set connection type
    if (response == "S" || response == "s") {
      serialComm = true;
    }
    else if (response == "W" || response == "w") {
      wifiComm = true;
    }
    else if (response == "B" || response == "b") {
      serialComm = true;
      wifiComm = true;
    }
    if (response != "N" && response != "n") {
      preferences.putBool("SerialComm", serialComm);
      preferences.putBool("WifiComm", wifiComm);
    }
  }

  // Connect to WiFi
  if (wifiComm) {
    WiFi.begin(ssid, password);
    for (int i = 0; WiFi.status() != WL_CONNECTED && i < 10; i++) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to WiFi!");
    }
    else if (!serialComm) {
      Serial.println("Wi-Fi connection failed. Switching to serial");
      preferences.putBool("serialComm", true);
    }
  }

  //Set server
  client.setServer(mqttServer, 1883);

  //MQTT server connect
  while (!client.connected()) {
    Serial.print("Connecting to MQTT Client...");
    if (client.connect("FabClient")) {
      Serial.println("connected to MQTT broker!");
    }
    else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait before retrying
    }
  }

  //Server setup
  client.setCallback(mqttCallback);

  //Set MQTT subscriptions
  client.subscribe(freqTopic.c_str());

  // Close Preferences
  preferences.end();
  leds[0] = CRGB::Aqua; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
  FastLED.show();       // Update LED
  delay(50);

  // Create Communication and Sensors FreeRTOS tasks
  // Initialize the mutex first
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL)
  {
    if (serialComm) {
      Serial.println("Failed to create mutex, halt!");
    }
    while (1)
      ; // Halt execution
  }
  // Create tasks : // Task function // Name of task // Stack size // Task parameters // Priority // Task handle // Core 1
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 10000, NULL, 1, &sensorTaskHandle, 0);    // Core 0
  xTaskCreatePinnedToCore(CommunicationTask, "CommTask", 10000, NULL, 1, &commTaskHandle, 1); // Core 1
  xTaskCreatePinnedToCore(ArduinoTask, "ArduinoTask", 2048, NULL, 1, &arduinoTaskHandle, 1);  // Core 1

  if (serialComm) {
      Serial.println("Mutex created. Tasks created. Leaving setup().");
  }
  
  leds[0] = CRGB::Black; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
  FastLED.show();        // Update LED - OFF
}

void loop()
{
  // Empty. Tasks are running independently.
}

// // MAIN Communication task
// void CommunicationTask(void *pvParameters)
// {
//   SensorData sensorDataArrayLocal;
//   sensorDataArrayLocal.identifier = sensorDataArray.identifier; // Sensor identifier char
//   sensorDataArrayLocal.valueCount = sensorDataArray.valueCount; // Up to 5 float values

//   int core = xPortGetCoreID();
//   Serial.print("CommunicationTask started on core ");
//   Serial.println(core);

//   for (;;)
//   { // Loop sequence

//     // Acquire mutex before reading shared data
//     if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
//     {
//       // for (int i = 0; i < 5; i++) {
//       //   // Send sensor data
//       //   Serial.print("Sensor ");
//       //   Serial.print(sensorDataArray[i].identifier);
//       //   Serial.print(" @ ");
//       //   Serial.print(sensorDataArray[i].timestamp);
//       //   Serial.print("ms: ");

//       //   for (int j = 0; j < sensorDataArray[i].valueCount; j++) {
//       //     Serial.print(sensorDataArray[i].values[j], 4); // 4 decimal places
//       //     if (j < sensorDataArray[i].valueCount - 1) {
//       //       Serial.print(", ");
//       //     }
//       //   }
//       //   Serial.println();
//       // }

//       sensorDataArrayLocal.timestamp = sensorDataArray.timestamp; // transfer timestamp to local struct
//       for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
//       {
//         sensorDataArrayLocal.values[j] = sensorDataArray.values[j]; // transfer values to local struct
//       }
//       xSemaphoreGive(xMutex);
//     }

//     Serial.print(sensorDataArrayLocal.identifier);
//     Serial.print(" ");
//     Serial.print(sensorDataArrayLocal.timestamp);
//     Serial.print(" ");

//     for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
//     {
//       Serial.print(sensorDataArrayLocal.values[j], 4); // 4 decimal places
//       if (j < sensorDataArrayLocal.valueCount - 1)
//       {
//         Serial.print(" ");
//       }
//     }
//     Serial.println();

//     // Delay for the next communication
//     vTaskDelay(communicationInterval / portTICK_PERIOD_MS);
//   }
// }
void CommunicationTask(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(300)); // Wait for last Serial write in setup and for first sensor conversion (resolution is 10ms)
  SensorData sensorDataArrayLocal;
  sensorDataArrayLocal.identifier = sensorDataArray.identifier;   // Sensor identifier char
  sensorDataArrayLocal.sensoralias = sensorDataArray.sensoralias; // Alias name
  sensorDataArrayLocal.valueCount = sensorDataArray.valueCount;   // Up to 5 float values

  int core = xPortGetCoreID();
  if (serialComm) {
      Serial.print("CommunicationTask started on core ");
      Serial.println(core);
    }

  // Initial loop frequency in milliseconds (default to 1000ms)
  uint32_t loopFrequencyMs = 1000;

  // Flag to control sending sensor data
  bool sendSensorData = false;

  // Variables for timing using millis()
  uint32_t previousMillis = 0;

  // Buffer for incoming serial data
  String inputString = "";
  bool inputComplete = false;

  for (;;)
  { // Free-running loop
    uint32_t currentMillis = millis();

    // Handle serial input as soon as it becomes available
    while (Serial.available() > 0)
    {
      char inChar = (char)Serial.read();
      if (inChar == '\n' || inChar == '\r')
      {
        inputComplete = true;
        break;
      }
      else
      {
        inputString += inChar;
      }
    }

    if (inputComplete)
    {
      inputComplete = false;
      inputString.trim(); // Remove any leading/trailing whitespace

      if (inputString == "?")
      {
        // Respond with identifier and stop sending sensor data
        Serial.println(sensorDataArrayLocal.sensoralias);
        sendSensorData = false;
      }
      else if (inputString == "A")
      {
        // Respond with "Arduino"
        Serial.println("Arduino"); // temporar
        // Serial2.println("Arduino2"); //temporar
      }
      else
      {
        // Try to parse the input as an integer
        int receivedNumber = inputString.toInt();
        if (receivedNumber > 0 || inputString == "0") // because toInt() to not a number returns a 0
        {
          if (receivedNumber == 0)
          {
            // Stop sending sensor data
            sendSensorData = false;
            leds[0] = CRGB::Black;
            FastLED.show(); //  LED Off
          }
          else
          {
            // Set new loop frequency (receivedNumber is in Hz)
            loopFrequencyMs = 1000 / receivedNumber;
            sendSensorData = true;          // Resume sending sensor data
            previousMillis = currentMillis; // Reset timing

            leds[0] = CRGB::Cyan; // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
            FastLED.show();       // Update LED
          }
          if (wifiComm && client.connected()) {
              char x[16];  // plenty for an integer
              snprintf(x, sizeof(x), "%d", receivedNumber);
              client.publish(freqTopic.c_str(), x, true);
          }
        }
        else
        {
          Serial.println("Invalid input received.");
        }
      }
      inputString = ""; // Clear the input buffer
    }
    if (wifiComm && strcmp(recTopic, freqTopic.c_str()) == 0) {
      int receivedNumber = atoi(recMessage);  // convert string to int

      if (receivedNumber >= 0 && receivedNumber < 100) {
          loopFrequencyMs = 1000 / receivedNumber;
          sendSensorData = true;
          previousMillis = currentMillis;

          leds[0] = CRGB::Cyan;
          FastLED.show();
      }

      // clear topic so we don't process it twice
      recTopic[0] = '\0';
    }


    // Check if it's time to send sensor data
    if (sendSensorData && (currentMillis - previousMillis >= loopFrequencyMs))
    {
      previousMillis = currentMillis; // Update timing

      // Acquire mutex before reading shared data
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        sensorDataArrayLocal.timestamp = sensorDataArray.timestamp; // transfer timestamp to local struct
                                                                    // sensorDataArrayLocal.valueCount = sensorDataArray.valueCount;
        for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
        {
          sensorDataArrayLocal.values[j] = sensorDataArray.values[j]; // transfer values to local struct
        }
        xSemaphoreGive(xMutex);
      }

      // Send sensor data over serial
      if (serialComm) {
        Serial.print(sensorDataArrayLocal.identifier);
        Serial.print(" ");
        Serial.print(sensorDataArrayLocal.timestamp);
        Serial.print(" ");

        for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
        {
          Serial.print(sensorDataArrayLocal.values[j], 2); // 2 decimal places
          if (j < sensorDataArrayLocal.valueCount - 1)
          {
            Serial.print(" ");
          }
          Serial.println();
        }
      }
      if (wifiComm) {
        char msg[256];  // big enough buffer
        snprintf(msg, sizeof(msg), "%c %u", 
                sensorDataArrayLocal.identifier, 
                sensorDataArrayLocal.timestamp);

        for (int j = 0; j < sensorDataArrayLocal.valueCount; j++) {
            char buf[32];
            snprintf(buf, sizeof(buf), " %.2f", sensorDataArrayLocal.values[j]);
            strncat(msg, buf, sizeof(msg) - strlen(msg) - 1);
        }

        // publish full message once
        if (!client.publish(dataTopic.c_str(), msg, true)) {
            Serial.println("MQTT publish failed!");
        }
        client.loop();
      }

    }

    // Yield to allow lower-priority tasks to run
    taskYIELD();
  }
}

// Main Arduino communication Task
void ArduinoTask(void *pvParameters)
{
  SensorData sensorDataArrayLocal;
  sensorDataArrayLocal.identifier = sensorDataArray.identifier; // Sensor identifier char
  sensorDataArrayLocal.valueCount = sensorDataArray.valueCount; // Up to 5 float values
  // Buffer for incoming serial2 data
  String inputString = "";
  bool inputComplete = false;

  for (;;)
  { // Free-running loop
    // Handle serial input as soon as it becomes available
    while (Serial2.available() > 0)
    {
      char inChar = (char)Serial2.read();
      if (inChar == '\n' || inChar == '\r')
      {
        inputComplete = true;
        break;
      }
      else
      {
        inputString += inChar;
      }
    }

    if (inputComplete)
    {
      inputComplete = false;
      inputString.trim(); // Remove any leading/trailing whitespace

      if (inputString == "?") // test incoming characters
      {
        // Respond with identifier
        Serial2.println(sensorDataArrayLocal.identifier);
      }
      else if (inputString == "A")
      { // Respond with data
        // Acquire mutex before reading shared data
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          sensorDataArrayLocal.timestamp = sensorDataArray.timestamp; // transfer timestamp to local struct
                                                                      // sensorDataArrayLocal.valueCount = sensorDataArray.valueCount;
          for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
          {
            sensorDataArrayLocal.values[j] = sensorDataArray.values[j]; // transfer values to local struct
          }
          xSemaphoreGive(xMutex);
        }

        // Send sensor data over serial
        for (int j = 0; j < sensorDataArrayLocal.valueCount; j++)
        {
          Serial2.print(sensorDataArrayLocal.values[j], 2); // 2 decimal places
          if (j < sensorDataArrayLocal.valueCount - 1)
          {
            Serial2.print(" ");
          }
        }
        Serial2.println();
        // Blink Led
        if (leds[0] == CRGB::Black)
        {
          leds[0] = CRGB::Yellow;         // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
          FastLED.show();               // Update LED
          vTaskDelay(pdMS_TO_TICKS(2)); // Delay for 2 milliseconds,  ATTENTION this will slow the data rate
          leds[0] = CRGB::Black;        // Led basic colors available Red Orange Gold Yellow Green Aqua Cyan Teal Blue Violet Purple Magenta Pink White Silver Black
          FastLED.show();
        }
      }

      else
      {
        Serial2.println("Invalid input received.");
      }
      inputString = ""; // Clear the input buffer
    }
    // Yield to allow lower-priority tasks to run and blink LED
    taskYIELD();
    vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 1 milliseconds
  }
}

// MAIN Sensor task
void SensorTask(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(20)); // Wait 20ms for last Serial write in setup

  int core = xPortGetCoreID();
  Serial.print("SensorTask started on core ");
  Serial.println(core);

  // SENSORS Initialisation
  // const int sensorPins[5] = {34, 35, 32, 33, 25}; // Adjust as necessary
  // const char sensorIdentifiers[5] = {'A', 'B', 'C', 'D', 'E'};
  //
  // for (int i = 0; i < 5; i++)
  // {
  //   pinMode(sensorPins[i], INPUT);
  // }
  if (sensorDataArray.identifier == 'L') // Light
  {
    if (!veml.begin())
    {
      Serial.println("Sensor not found, halting.");
      deleteTasks(&commTaskHandle, &arduinoTaskHandle);
      while (1) // halt
        ;
    }
  }
  else if (sensorDataArray.identifier == 'T') // Temperature
  {
    // Setup SPI pins
    SPI.begin(MAX31865_SCK_PIN, MAX31865_MISO_PIN, MAX31865_MOSI_PIN, MAX31865_CS_PIN);
    // Configure Chip Select and RDY pins
    pinMode(MAX31865_CS_PIN, OUTPUT);
    pinMode(MAX31865_RDY_PIN, INPUT);
    // Set MAX31865 to continuous conversion mode with 60Hz filter (for faster sampling)
    SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE1));
    digitalWrite(MAX31865_CS_PIN, LOW);
    SPI.transfer(0x80); // Write to config register
    SPI.transfer(0xC0); // 3-wire RTD, 60Hz filter, continuous conversion mode
    digitalWrite(MAX31865_CS_PIN, HIGH);
    SPI.endTransaction();
    // Take an initial reading for proper interpolation
    TempeLastTemperature = TempeReadTemperature();
    TempeLastSampleTime = millis();
    TempeNextInterpolationTime = TempeLastSampleTime + TempeInterpolationInterval; // Schedule first interpolation
  }
  else if (sensorDataArray.identifier == 'P') // Pressure
  {
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C communication with specified SDA and SCL pins
  }
  else if (sensorDataArray.identifier == 'D') // Distance
  {
    Wire.begin();
    if (distanceSensor.begin() != 0) // Begin returns 0 on a good init
    {
      Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
      deleteTasks(&commTaskHandle, &arduinoTaskHandle);
      while (1)
        ;
    }
    distanceSensor.setDistanceModeLong();         // Use short mode if aiming for 50Hz sampling rate
    distanceSensor.setTimingBudgetInMs(33);       // 20ms timing budget
    distanceSensor.setIntermeasurementPeriod(33); // Must be greater or equal to timing budget
    distanceSensor.startRanging();
  }
  else if (sensorDataArray.identifier == 'C') // Current
  {
    if (!ads1.begin(0x48))
    { // ADS1015 with ADDR pin LOW (GND)
      Serial.println("Failed to initialize ADC#1, halt!");
      deleteTasks(&commTaskHandle, &arduinoTaskHandle);
      while (1)
        ;
    }
    if (!ads2.begin(0x49))
    { // ADS1015 with ADDR pin HIGH (VDD)
      Serial.println("Failed to initialize ADC#2, halt!");
      deleteTasks(&commTaskHandle, &arduinoTaskHandle); // kill tasks
      while (1)
        ;
    }
    ads1.setGain(GAIN_FOUR); // Set the gains (GAIN_SIXTEEN ±0.256V range, maximum gain)
    ads2.setGain(GAIN_SIXTEEN);
  }
  else // unknown type handling
  {
  }

  for (;;)
  { // Loop sequence
    currentTime = millis();

    if (sensorDataArray.identifier == 'L') // Light
    {
      sensorVal[0] = veml.readLux(VEML_LUX_AUTO); // lux value
      SensorSemaphoreSend();                      // Update the global sensor struct
    }
    else if (sensorDataArray.identifier == 'T') // Temperature
    {
      // Poll the RDY pin to check if new data is ready
      if (digitalRead(MAX31865_RDY_PIN) == LOW)
      {
        // Store the previous temperature before updating
        TempeLastTemperature = TempeCurrentTemperature;
        TempeLastSampleTime = millis(); // Store the time of the actual sample
        // Read the new temperature when RDY pin goes low
        TempeCurrentTemperature = TempeReadTemperature();
        // // Print the real temperature reading for separate use
        // Serial.print("Real Time (ms): ");
        // Serial.print(TempeLastSampleTime);
        // Serial.print(" - Real Temperature: ");
        // Serial.println(TempeCurrentTemperature);
        // Mark that new real data is available
        TempeNewRealDataAvailable = true;
      }
      // Interpolation Task: Perform interpolation exactly every 10ms, regardless of real data timing
      unsigned long TempeCurrentTime = millis();
      if (TempeCurrentTime >= TempeNextInterpolationTime)
      {
        // Calculate how much time has passed since the last real sample
        unsigned long TempeElapsedTime = TempeCurrentTime - TempeLastSampleTime;
        // Linear interpolation between the last and current real temperature
        float TempeInterpolatedTemp = TempeLastTemperature + (TempeCurrentTemperature - TempeLastTemperature) * (float)TempeElapsedTime / 16.0;
        sensorVal[0] = TempeInterpolatedTemp; // Temperature value to be sent
        currentTime = TempeCurrentTime;       // timestamp to be sent
        SensorSemaphoreSend();                // Update the global sensor struct
        // // Print the interpolated temperature
        // Serial.print("Interpolated Time (ms): ");
        // Serial.print(TempeNextInterpolationTime);
        // Serial.print(" - Interpolated Temperature: ");
        // Serial.println(TempeInterpolatedTemp);

        // Schedule the next interpolation
        TempeNextInterpolationTime += TempeInterpolationInterval;
      }
    }
    else if (sensorDataArray.identifier == 'P') // Pressure
    {
      sensorVal[0] = readPressure(); // call function for pressure value
      SensorSemaphoreSend();         // Update the global sensor struct
    }
    else if (sensorDataArray.identifier == 'D') // Distance
    {
      VLcurrentTime = millis(); // Get the current time in ms

      // Check if new data is available
      if (distanceSensor.checkForDataReady())
      {
        // Calculate time difference since last valid sample
        float timeDiff = (VLcurrentTime - VLlastSampleTime) / 1000.0; // Convert ms to seconds
        VLlastSampleTime = VLcurrentTime;                             // Update last sample time
        currDistance = distanceSensor.getDistance();                  // Read current distance
        distanceSensor.clearInterrupt();                              // Clear the interrupt for the next reading
        // Apply exponential smoothing
        VLsmoothedDistance = VLsmoothingFactor * currDistance + (1 - VLsmoothingFactor) * VLsmoothedDistance;
        if (timeDiff > 0)
        { // Calculate speed (smoothed distance difference / time difference)
          currSpeed = (VLsmoothedDistance - prevDistance) / timeDiff;
        }
        else
        {
          currSpeed = 0;
        }
        if (timeDiff > 0)
        { // Calculate acceleration (speed difference / time difference)
          acceleration = (currSpeed - prevSpeed) / timeDiff;
        }
        else
        {
          acceleration = 0;
        }
        // Output the smoothed distance, speed, and acceleration
        sensorVal[0] = VLsmoothedDistance; // position value
        sensorVal[1] = currSpeed;          // speed value
        sensorVal[2] = acceleration;       // acceleration value
        SensorSemaphoreSend();             // Update the global sensor struct
        // Store the current smoothed distance and speed as previous for the next loop
        prevDistance = VLsmoothedDistance;
        prevSpeed = currSpeed;
      }
    }
    else if (sensorDataArray.identifier == 'C') // Current
    {
      // Read differential inputs from both ADCs
      int16_t adc1_diff1 = ads1.readADC_Differential_0_1(); // Reading differential A0 - A1 from ads1
      int16_t adc1_diff2 = ads1.readADC_Differential_2_3(); // Reading differential A2 - A3 from ads1
      int16_t adc2_diff = ads2.readADC_Differential_0_1();  // Reading differential A0 - A1 from ads2
      float volts1 = -ads1.computeVolts(adc1_diff2)*1.0811;        // Convert to voltage (based on the gain setting) Range from 0 to +/-1V, (-) sign du to opamp
      if (abs(volts1) > 1)
      { // then use the 1/10 attenuator readingA0 - A1 from ads1  Range from */-1V to +/-10V (10mV resolution)
        volts1 = ads1.computeVolts(adc1_diff1) * 11.062;
      }
      float volts2 = ads2.computeVolts(adc2_diff) / 0.0625; // convert to current based on two parallel 0.125 ohm shunt
      sensorVal[0] = volts1;                                // voltage value
      sensorVal[1] = volts2;                                // current value
      SensorSemaphoreSend();                                // Update the global sensor struct
    }
    else // unknown type handling
    {
    }

    taskYIELD(); // give control to RTOS
                 // vTaskDelay(pdMS_TO_TICKS(1));  // Delay for 1 milliseconds, otherwise WDT crash

    if (millis() % 4000 == 0)
    { // every 4 second put a short delay for WDT
      // Perform the 1ms delay
      vTaskDelay(pdMS_TO_TICKS(1)); // Delay for 1 millisecond
    }

    // // Delay for the next reading
    // vTaskDelay(sensorReadInterval / portTICK_PERIOD_MS);
  }
}

// FUNCTIONS
// Function to acquire mutex before modifying shared data, then transfer to global struct
void SensorSemaphoreSend()
{
  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
  {
    sensorDataArray.timestamp = currentTime; // transfer timestamp to global struct
    for (int j = 0; j < sensorDataArray.valueCount; j++)
    {
      sensorDataArray.values[j] = sensorVal[j]; // transfer values to global struct
    }
    xSemaphoreGive(xMutex);
  }
}

// Function to delete both CommunicationTask and ArduinoTask from SensorTask
void deleteTasks(TaskHandle_t *commTaskHandle, TaskHandle_t *arduinoTaskHandle)
{
  // Stop and delete the CommunicationTask
  if (*commTaskHandle != NULL)
  {
    vTaskDelete(*commTaskHandle); // Delete the CommunicationTask
    *commTaskHandle = NULL;       // Clear the handle
    Serial.println("CommunicationTask deleted.");
  }

  // Stop and delete the ArduinoTask
  if (*arduinoTaskHandle != NULL)
  {
    vTaskDelete(*arduinoTaskHandle); // Delete the ArduinoTask
    *arduinoTaskHandle = NULL;       // Clear the handle
    Serial.println("ArduinoTask deleted.");
  }

  // Optionally, delete the task that called this function
  vTaskDelete(NULL); // Delete the calling task itself
}

// Helper function to check if a value is within a fixed voltage tolerance
bool withinTolerance(float measured, float expected, float tolerance)
{
  float lowerBound = expected - tolerance;
  float upperBound = expected + tolerance;
  return (measured >= lowerBound && measured <= upperBound);
}

float TempeReadTemperature()
{
  uint16_t TempeRtdData;
  uint16_t TempeRtd;

  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE1));
  digitalWrite(MAX31865_CS_PIN, LOW);

  SPI.transfer(0x01); // Read RTD MSB
  TempeRtd = SPI.transfer(0x00) << 8;
  TempeRtd |= SPI.transfer(0x00); // Read RTD LSB

  digitalWrite(MAX31865_CS_PIN, HIGH);
  SPI.endTransaction();

  TempeRtd >>= 1; // The data is in bits 15:1
  TempeRtdData = TempeRtd;

  // Calculate temperature using the RTD resistance formula
  // Assuming PT100 and a reference resistor of 430 ohms
  float TempeRtdResistance = (TempeRtdData / 32768.0) * RREF;
  float TempeTemperature = (TempeRtdResistance - RNOMINAL) / RNOMINAL / 0.00385; // PT100 sensitivity

  return TempeTemperature;
}

float readPressure()
{
  // // Start a combined conversion (pressure and temperature)
  // Wire.beginTransmission(SENSOR_ADDR);
  // Wire.write(REG_CMD);
  // Wire.write(0x0A); // Combined conversion command
  // Wire.endTransmission();

  // // Wait for conversion to complete by polling the sensor
  // delay(5); // Short delay for conversion

  // // Poll to check if conversion is done by reading the CMD register
  // byte status = 1;
  // while (status & 0x08)
  // { // Check the conversion ready flag in the status byte
  //   Wire.beginTransmission(SENSOR_ADDR);
  //   Wire.write(REG_CMD);
  //   Wire.endTransmission();
  //   Wire.requestFrom(SENSOR_ADDR, 1);
  //   if (Wire.available())
  //   {
  //     status = Wire.read();
  //   }
  //   delay(2); // Short delay between polls
  // }

  // // Read the pressure data (24-bit value)
  // Wire.beginTransmission(SENSOR_ADDR);
  // Wire.write(REG_PRESSURE_MSB);
  // Wire.endTransmission();
  // Wire.requestFrom(SENSOR_ADDR, 3); // Request 3 bytes of pressure data

  // long pressure_adc = 0;
  // if (Wire.available() == 3)
  // {
  //   byte pressure_H = Wire.read(); // MSB
  //   byte pressure_M = Wire.read(); // CSB
  //   byte pressure_L = Wire.read(); // LSB

  //   // Combine the three bytes into a single 24-bit integer
  //   pressure_adc = (long)pressure_H << 16 | (long)pressure_M << 8 | (long)pressure_L;

  //   // Calculate pressure value based on the ADC output
  //   float pressure;
  //   if (pressure_adc & 0x800000)
  //   {                                                  // Negative pressure
  //     pressure = (pressure_adc - 16777216L) / K_VALUE; // Use K=16 for 0 to 500 kPa range
  //   }
  //   else
  //   { // Positive pressure
  //     pressure = pressure_adc / K_VALUE;
  //   }

  // Start a combined conversion (pressure and temperature)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(REG_CMD);
  Wire.write(0x0A); // Combined conversion command
  if (Wire.endTransmission() != 0)
  {
    // Error in I2C communication
    return NAN; // Return Not-A-Number to indicate an error
  }

  // Wait for conversion to complete by polling the sensor
  delay(5); // Short delay for conversion

  // Poll to check if conversion is done by reading the CMD register
  byte status = 1;
  unsigned long startTime = millis();
  const unsigned long timeout = 100; // Timeout in milliseconds

  while (status & 0x08)
  { // Check the conversion ready flag in the status byte
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(REG_CMD);
    if (Wire.endTransmission() != 0)
    {
      // Error in I2C communication
      return NAN;
    }

    Wire.requestFrom(SENSOR_ADDR, (uint8_t)1);
    if (Wire.available())
    {
      status = Wire.read();
    }
    else
    {
      // Error: no data received
      return NAN;
    }

    // Check for timeout
    if (millis() - startTime > timeout)
    {
      // Timeout occurred
      return NAN;
    }
    delay(2); // Short delay between polls
  }

  // Read the pressure data (24-bit value)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(REG_PRESSURE_MSB);
  if (Wire.endTransmission() != 0)
  {
    // Error in I2C communication
    return NAN;
  }

  Wire.requestFrom(SENSOR_ADDR, (uint8_t)3); // Request 3 bytes of pressure data

  long pressure_adc = 0;
  if (Wire.available() == 3)
  {
    byte pressure_H = Wire.read(); // MSB
    byte pressure_M = Wire.read(); // CSB
    byte pressure_L = Wire.read(); // LSB

    // Combine the three bytes into a single 24-bit integer
    pressure_adc = ((long)pressure_H << 16) | ((long)pressure_M << 8) | (long)pressure_L;

    // Calculate pressure value based on the ADC output
    float pressure;
    if (pressure_adc & 0x800000)
    {                                                  // Negative pressure
      pressure = (pressure_adc - 16777216L) / K_VALUE; // Use K_VALUE based on your sensor range
    }
    else
    { // Positive pressure
      pressure = pressure_adc / K_VALUE;
    }

    return pressure;
  }
  else
  {
    // Error: insufficient data received
    return NAN; // Return Not-A-Number to indicate an error
  }
}