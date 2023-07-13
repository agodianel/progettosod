```c


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
char* mqttBroker = "192.168.147.103";
int mqttPort = 1883;
char* mqttClientId = "amebaClient";
char* mqttUsername = "ameba";
char* mqttPassword = "ameba";

// Topic definition
char* mqttSensorTopic = "sensor";
char* mqttSyncTopic = "timestamp";

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

// Variables used for reconnection WiFi & MQTT
long lastReconnectAttemptWIFI = 0;
long lastReconnectAttemptMQTT = 0;

// Control variables
boolean isFirst = true;
boolean isArrived = false;

// Variable to store first time sync
uint32_t temp = 0;

// Queues
QueueHandle_t sensorQueue;

// Binary semaphore
SemaphoreHandle_t binarySemaphore;

// Reconnect function for MQTT
boolean reconnect() {

  if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
    mqttClient.subscribe(mqttSyncTopic);
  }

  return mqttClient.connected();

}

// Callback function: receive timestamp as message from topic through MQTT and sync time on PCF8523
void callback(char* topic, byte* payload, unsigned int length) {
  // Control if it is first timestamp
  if (isFirst) {
    isFirst = false;
    if (strcmp(topic, mqttSyncTopic) == 0) {
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
      temp = timestamp;
      Serial.println(temp);
      isArrived = true;
    } 
  // Control if it is not first timestamp  
  } else if (!isFirst) {
    if (strcmp(topic, mqttSyncTopic) == 0) {
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
        DateTime dt = DateTime(timestamp);
        rtc.adjust(dt);
    }
  }
}

// WiFi initialization procedure
void wifiInit() {

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.flush();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      delay(5000);
    }
  }
  Serial.println("Connected to WiFi!");

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

  // Initialize PCF8523
  if (!rtc.begin()) {
    Serial.println("Failed to initialize PCF8523!");
    while (1);
  } else {
    Serial.println("PCF8523 initialized!");
    rtc.start();
  }
  
}

// Function that receive timestamp and set rtc sensor for the first time
void syncFT() {
  while (isFirst) {
    if (!reconnect()) {;}
    // Control if first timestamp is arrived
    if (isArrived) {
      DateTime dt = DateTime(temp);
      rtc.adjust(dt);
      Serial.println("Sync done!");
      isFirst = false;
    }
    
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
    xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY);

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
    Serial.println((int)(sensorData.hr));
    Serial.println(sensorData.spo2);
    Serial.println(sensorData.tvoc);
    Serial.println(sensorData.eco2);
    Serial.println(sensorData.currentTime);

    // Send sensors data through queue
    xQueueSend(sensorQueue, &sensorData, 0);

    // Give semaphore
    xSemaphoreGive(binarySemaphore);

    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // 100 ms
  }

}

/* 
MQTT task: 
  1. Check if WiFi & MQTT are connected
  2. Receive timestamp from MQTT topic and sync time on PCF8523
  3. Receive sensors data through queue from Sensor task and send it as message to MQTT topic
*/

void mqttTask(void* params) {

  (void) params;
  SensorData sensorData;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    // Take semaphore
    xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY);

    // Loop until WiFi is connected
    while (WiFi.status() != WL_CONNECTED) {

      long now = millis();

      if (now - lastReconnectAttemptWIFI >= 5000) {
        lastReconnectAttemptWIFI = now;

        // Attempt to  WiFi every 5 seconds
        if (WiFi.begin(ssid, password) == WL_CONNECTED) {
          lastReconnectAttemptWIFI = 0;
        }
      }
    }

    // // Loop until MQTT is connected
    while (!mqttClient.connected()) {
      long now = millis();

      if (now - lastReconnectAttemptMQTT > 5000) {
        lastReconnectAttemptMQTT = now;

        // Attempt to  MQTT every 5 seconds
        if (reconnect()) {
          mqttClient.loop();
          lastReconnectAttemptMQTT = 0;
        }
      }
    }

    // Receive sensors data from queue and publish a message on topic MQTT
    if (xQueueReceive(sensorQueue, &sensorData, 0) == pdPASS) {
      char sensorMess[100];
      snprintf(sensorMess, sizeof(sensorMess), "%d,%d,%d,%d,%d", sensorData.tvoc, sensorData.eco2, (int)sensorData.hr, sensorData.spo2, sensorData.currentTime);
      mqttClient.publish(mqttSensorTopic, sensorMess);
      mqttClient.loop();
    }

    // Give semaphore
    xSemaphoreGive(binarySemaphore);

    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // 100 ms
  }

}

void setup() {

  Serial.begin(115200);

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(callback);

  // Connect to WiFi
  wifiInit();

  // Initialize sensors
  sensorInit();

  Serial.println("Wait for timestamp");

  syncFT();
   
  // Create queues
  sensorQueue = xQueueCreate(1, sizeof(SensorData));

  // Create binary semaphore
  binarySemaphore = xSemaphoreCreateBinary();

  // Create tasks with their priorities
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 2, NULL);
  xTaskCreate(mqttTask, "MqttTask", 4096, NULL, 3, NULL);

  // Give semaphore
  xSemaphoreGive(binarySemaphore);

}

// Unused
void loop() {}


```