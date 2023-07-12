
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
char* mqttBroker = "192.168.108.103";
int mqttPort = 1883;
char* mqttClientId = "amebaClient";
char* mqttUsername = "ameba";
char* mqttPassword = "ameba";

// Topic creation
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

// Task handles (unused)
TaskHandle_t sensorTaskHandle;
TaskHandle_t mqttTaskHandle;
TaskHandle_t syncTaskHandle;

// Queues
QueueHandle_t sensorQueue;
QueueHandle_t syncQueue;

// Semaphore
SemaphoreHandle_t sensorSemaphore;

// Reconnect function for MQTT
boolean reconnect() {

  if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword)) {
    mqttClient.subscribe(mqttSyncTopic);
    mqttClient.loop();
  }

  return mqttClient.connected();

}

// Callback function: receive timestamp from topic through MQTT and send it through a queue
void callback(char* topic, byte* payload, unsigned int length) {

  // Receive timestamp from topic
  if (strcmp(topic, mqttSyncTopic) == 0) {
    char mqttMess[100];
    memcpy(mqttMess, payload, length);
    mqttMess[length] = '\0';
    String mess = String(mqttMess);
    uint32_t timestamp = mess.toInt();

    // Send timestamp through queue
    xQueueSend(syncQueue, &timestamp, 0);
  }

}

// WiFi initialization procedure
void wifiInit() {

  // Connect to WiFi
  WiFi.begin(ssid, password);
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
    while (!pox.begin()) delay(500);
  } else {
    Serial.println("MAX30102 initialized!");
  }
  pox.setIRLedCurrent(MAX30102_LED_CURR_7_6MA);

  // Initialize SGP30
  if (!sgp30.begin()) {
    Serial.println("Failed to initialize SGP30!");
    while (!sgp30.begin()) delay(500);
  } else {
    Serial.println("SGP3O initialized!");
  }

  // Initialize PCF8523
  if (!rtc.begin()) {
    Serial.println("Failed to initialize PCF8523!");
    while (!rtc.begin()) delay(500);
  } else {
    Serial.println("PCF8523 initialized!");
  }
  rtc.start();

}

// Sensor task: read data from sensors and send them through queue to MQTT task
void sensorTask(void* params) {

  (void) params;
  SensorData sensorData;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    // Take semaphore
    xQueueSemaphoreTake(sensorSemaphore, portMAX_DELAY);

    // Read sensors data
    pox.update();
    sensorData.hr = pox.getHeartRate();
    sensorData.spo2 = pox.getSpO2();

    if (!sgp30.IAQmeasure()) {
      Serial.println("Failed to measure IAQ!");

      while (!sgp30.IAQmeasure()) delay(500);

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
    xSemaphoreGive(sensorSemaphore);

    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // 100 ms
  }

}

/* 
MQTT task: 
            1. Check if WiFi & MQTT are connected
            2. Receive timestamp from MQTT topic and send it through a queue
            3. Send sensors data as message to MQTT topic
*/
void mqttTask(void* params) {

  (void) params;
  SensorData sensorData;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    // Take semaphore
    xQueueSemaphoreTake(sensorSemaphore, portMAX_DELAY);

    // Loop until WiFi is connected
    while (WiFi.status() != WL_CONNECTED) {

      long now = millis();

      if (now - lastReconnectAttemptWIFI > 5000) {
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
    xSemaphoreGive(sensorSemaphore);

    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));  // 200 ms
  }

}

// Sync Task: receive timestamp from queue and set rtc sensor
void syncTask(void* params) {

  (void) params;
  uint32_t timestamp;
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    // Take semaphore
    xSemaphoreTake(sensorSemaphore, portMAX_DELAY);

    // Receive timestamp from queue and set time on PCF8523
    if (xQueueReceive(syncQueue, &timestamp, 0) == pdPASS) {
      DateTime dt = DateTime(timestamp);
      rtc.adjust(dt);
    }

    // Give semaphore
    xSemaphoreGive(sensorSemaphore);
    
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

  // Create queues
  sensorQueue = xQueueCreate(1, sizeof(SensorData));
  syncQueue = xQueueCreate(1, sizeof(uint32_t));

  // Create mutex
  sensorSemaphore = xSemaphoreCreateMutex();

  // Create tasks with their priorities
  xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle);
  xTaskCreate(mqttTask, "MqttTask", 4096, NULL, 3, &mqttTaskHandle);
  xTaskCreate(syncTask, "SyncTask", 4096, NULL, 2, &syncTaskHandle);


  // Give semaphore
  xSemaphoreGive(sensorSemaphore);

}

// Unused
void loop() {}