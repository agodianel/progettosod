
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>
#include "MAX30102_PulseOximeter.h"
#include <RTClib.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
char* ssid = "appz";
char* password = "ago12345";

// MQTT broker information
char* mqttBroker = "192.168.228.103";
int mqttPort = 1883;
char* mqttClientId = "amebaClient";
char* mqttUsername = "ameba";
char* mqttPassword = "ameba";

// Topic definition
char* mqttSensorTopic = "sensor";
char* mqttSyncTopic = "timestamp";
char* mqttHistoryTopic = "history";

// Sensor objects
Adafruit_SGP30 sgp30;
PulseOximeter pox;
RTC_PCF8523 rtc;  

// Wifi & Mqtt objects
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Struct to contain sensor data
typedef struct {
  uint16_t tvoc;
  uint16_t eco2;
  float hr;
  uint8_t spo2;
  uint32_t currentTime;
} SensorData;

// Control variable
boolean isFirst;
boolean mqttConnected;

// Predefined delay time
TickType_t delaySensor = pdMS_TO_TICKS(100);
TickType_t delayMqttC = pdMS_TO_TICKS(1000);
TickType_t delayMqttDc = pdMS_TO_TICKS(1000);
TickType_t delayEmpty = pdMS_TO_TICKS(100);

// Queues
QueueHandle_t sensorQueue;
QueueHandle_t historyQueue;

// Binary semaphore
SemaphoreHandle_t binarySemaphore;

// Handle MQTT 
void handleMqttConnection() {
  if (!mqttClient.connected()) {
    if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
      mqttConnected = true;
      mqttClient.subscribe(mqttSyncTopic);
      mqttClient.loop();
    } else {
      mqttConnected = false;
    }
  }
}

// Reconnect function for MQTT
boolean reconnectMQTT() {

  if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
    mqttClient.subscribe(mqttSyncTopic);
  }

  return mqttClient.connected();
}

// Callback function: receive timestamp as message from topic through MQTT and sync time on PCF8523
void callback(char* topic, byte* payload, unsigned int length) {
  
  if (strcmp(topic, mqttSyncTopic) == 0) {

    // Run only first time sync
    if (isFirst) {
      isFirst = false;
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
      DateTime dt = DateTime(timestamp);
      rtc.adjust(dt);
    
    // Run other times
    } else {
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
      DateTime dt = DateTime(timestamp);
      if (xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY) == pdTRUE) {
        rtc.adjust(dt);
        xSemaphoreGive(binarySemaphore);
      }
    }
  }
}

// WiFi initialization procedure
void wifiInit() {

  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nConnecting to WiFi...");
    WiFi.begin(ssid, password);
    delay(10000);
  }
  Serial.println("\n\nConnected to WiFi!\n");
}

// Sensors initialization procedure
void sensorInit() {

  // Initialize MAX30102
  if (!pox.begin()) {
    Serial.println("Failed to initialize MAX30102!");
    while (1);
  } else {
    Serial.println("MAX30102 initialized!");
  }
  pox.setIRLedCurrent(MAX30102_LED_CURR_7_6MA);

  // Initialize SGP30
  if (!sgp30.begin()) {
    Serial.println("Failed to initialize SGP30!");
    while (1);
  } else {
    Serial.println("SGP3O initialized!");
  }
}

// Function that receive timestamp and set rtc sensor for the first time
void syncFT() {

  // Initialize PCF8523
  if (!rtc.begin()) {
    Serial.println("Failed to initialize PCF8523!");
    while (1);
  } else {
    Serial.println("PCF8523 initialized!");
    rtc.start();
  }

  // Loop until first timestamp is received and meanwhile check if WiFi and MQTT are connected
  while (isFirst) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      delay(5000); 
    }
    if (!reconnectMQTT()) { delay(5000); }
    mqttClient.loop();
  }
}

// Sensor task: read data from sensors and send them through queue to MQTT task
void sensorTask(void* params) {

  (void) params;
  SensorData sensorData;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    
    // Take semaphore
    if (xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY) == pdTRUE) {
    
      // Read sensors data
      pox.update();
      sensorData.hr = pox.getHeartRate();
      sensorData.spo2 = pox.getSpO2();

      if (!sgp30.IAQmeasure()) {
        Serial.println("Failed to measure IAQ!");
        while (1);

      } else {
        sensorData.tvoc = sgp30.TVOC;
        sensorData.eco2 = sgp30.eCO2;
      }

      sensorData.currentTime = rtc.now().unixtime();

      // Give semaphore
      xSemaphoreGive(binarySemaphore);
    }
    
    // Send sensors data through queue
    xQueueSend(sensorQueue, &sensorData, 0);
    
    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, delaySensor); 
  }
}

/* 
  MQTT task: 
    1. Check if WiFi & MQTT are connected
    2. Receive timestamp from MQTT topic and sync time on PCF8523
    3. Receive sensors data through queue from Sensor task and send it as message to both MQTT topic
    4. If MQTT is disconnected save data into queue
*/

void mqttTask(void* params) {

  (void) params;
  SensorData sensorData;
  TickType_t interval;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    
    handleMqttConnection();

    // If MQTT is disconnected 
    if (!mqttConnected) {
      
      // Save data from sensor queue to history queue, if history queue is full overwrite the oldest value
      if (xQueueReceive(sensorQueue, &sensorData, 0) == pdPASS) {
        if (xQueueSend(historyQueue, &sensorData, 0) == errQUEUE_FULL) {
          xQueueReceive(historyQueue, &sensorData, 0);
        }
      }
      interval = delayMqttDc;

    // If MQTT is connected 
    } else {

      // Empty history queue as fast as possible
      if (xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY) == pdTRUE) {
        while (xQueueReceive(historyQueue, &sensorData, 0) == pdPASS) {
          char historyMess[100];
          snprintf(historyMess, sizeof(historyMess), "%d,%d,%d,%d,%d", sensorData.tvoc, sensorData.eco2, (int)sensorData.hr, sensorData.spo2, sensorData.currentTime);
          mqttClient.publish(mqttHistoryTopic, historyMess);
          mqttClient.loop();
          vTaskDelay(delayEmpty);
        }
        xSemaphoreGive(binarySemaphore);
      }

      // Publish data on both topic
      if (xQueueReceive(sensorQueue, &sensorData, 0) == pdPASS) {
        char sensorMess[100];
        snprintf(sensorMess, sizeof(sensorMess), "%d,%d,%d,%d,%d", sensorData.tvoc, sensorData.eco2, (int)sensorData.hr, sensorData.spo2, sensorData.currentTime); 
        mqttClient.publish(mqttSensorTopic, sensorMess);
        mqttClient.publish(mqttHistoryTopic, sensorMess);
        mqttClient.loop();
      }
      
      interval = delayMqttC;
    }
    
    mqttClient.loop();
    vTaskDelayUntil(&xLastWakeTime, interval); 
  }
}

void setup() {

  Serial.begin(115200);
  isFirst = true;
  mqttConnected = true;

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(callback);

  // Connect to WiFi
  wifiInit();
  
  Serial.println("\nWait for timestamp...\n");
  syncFT();

  // Initialize sensors
  sensorInit();

  // Create queues
  sensorQueue = xQueueCreate(1, sizeof(SensorData));
  historyQueue = xQueueCreate(10, sizeof(SensorData));

  // Create binary semaphore
  binarySemaphore = xSemaphoreCreateBinary();

  // Create tasks with their priorities
  xTaskCreate(sensorTask, "SensorTask", 8192, NULL, 1, NULL);
  xTaskCreate(mqttTask, "MqttTask", 8192, NULL, 1, NULL);

  // Give semaphore
  xSemaphoreGive(binarySemaphore);
}

// Unused
void loop() {}
