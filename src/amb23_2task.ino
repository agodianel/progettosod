
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
char* ssid = "****";
char* password = "****";

// MQTT broker information
char* mqttBroker = "****";
int mqttPort = 1883;
char* mqttClientId = "****";
char* mqttUsername = "****";
char* mqttPassword = "****";

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
boolean mqttConnected = false;

// Predefined delay time
TickType_t delaySensor = pdMS_TO_TICKS(100);
TickType_t delayMqttC = pdMS_TO_TICKS(1000);
TickType_t delayMqttDc = pdMS_TO_TICKS(1000);

// Queues
QueueHandle_t sensorQueue;
QueueHandle_t historyQueue;

// Binary semaphore
SemaphoreHandle_t binarySemaphore;

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
    if (isFirst) {
      isFirst = false;
      Serial.println("Message received!");
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
      DateTime dt = DateTime(timestamp);
      rtc.adjust(dt);
      Serial.println("Sync done!");
    } else {
      Serial.println("Message received!");
      char mqttMess[100];
      uint32_t timestamp;
      memcpy(mqttMess, payload, length);
      mqttMess[length] = '\0';
      String mess = String(mqttMess);
      timestamp = mess.toInt();
      DateTime dt = DateTime(timestamp);
      xQueueSemaphoreTake(binarySemaphore, 0);
      rtc.adjust(dt);
      xSemaphoreGive(binarySemaphore);
      Serial.println("Sync done!");
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
    xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY);
    
    pox.update();
    //Serial.println("Sensor start!");

    // Read sensors data
    
    sensorData.hr = pox.getHeartRate();
    // sensorHistory.hr = pox.getHeartRate();
    sensorData.spo2 = pox.getSpO2();
    //sensorHistory.spo2 = pox.getSpO2();

    if (!sgp30.IAQmeasure()) {
      Serial.println("Failed to measure IAQ!");
      while (1);

    } else {
      sensorData.tvoc = sgp30.TVOC;
      //sensorHistory.tvoc = sgp30.TVOC;
      sensorData.eco2 = sgp30.eCO2;
      //sensorHistory.eco2 = sgp30.eCO2;
    }

    sensorData.currentTime = rtc.now().unixtime();
    //sensorHistory.currentTime = rtc.now().unixtime();

    
    /*
    Serial.println((int)(sensorData.hr));
    Serial.println(sensorData.spo2);
    Serial.println(sensorData.tvoc);
    Serial.println(sensorData.eco2);
    Serial.println(sensorData.currentTime);
    */
    xSemaphoreGive(binarySemaphore);
    // Send sensors data through queue
    xQueueSend(sensorQueue, &sensorData, 0);
    //xQueueSend(historyQueue, &sensorHistory, 0);

    // Give semaphore
    

  
    //Serial.println("Data sent!");

    //Serial.println("Sensor stop!");

    // Block task for a fixed period
    vTaskDelayUntil(&xLastWakeTime, delaySensor);  // 100 ms
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
  TickType_t interval;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    
    
    handleMqttConnection();

    if (mqttConnected) {
      xQueueSemaphoreTake(binarySemaphore, portMAX_DELAY);
      
      while (uxQueueMessagesWaiting(historyQueue) > 0) {
        TickType_t xLastWakeTime = xTaskGetTickCount();
        if (xQueueReceive(historyQueue, &sensorData, 0) == pdPASS) {
          char historyMess[100];
          snprintf(historyMess, sizeof(historyMess), "%d,%d,%d,%d,%d", sensorData.tvoc, sensorData.eco2, (int)sensorData.hr, sensorData.spo2, sensorData.currentTime);
          mqttClient.publish(mqttHistoryTopic, historyMess);
          mqttClient.loop();
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
        }
      }
      xSemaphoreGive(binarySemaphore);

      if (xQueueReceive(sensorQueue, &sensorData, 0) == pdPASS) {
        char sensorMess[100];
        snprintf(sensorMess, sizeof(sensorMess), "%d,%d,%d,%d,%d", sensorData.tvoc, sensorData.eco2, (int)sensorData.hr, sensorData.spo2, sensorData.currentTime); 
        mqttClient.publish(mqttSensorTopic, sensorMess);
        mqttClient.publish(mqttHistoryTopic, sensorMess);
        mqttClient.loop();
      }
      
      interval = delayMqttC;
    } else {
      
      if (xQueueReceive(sensorQueue, &sensorData, 0) == pdPASS) {
        xQueueSend(historyQueue, &sensorData, 0);
      }
      
      
      interval = delayMqttDc;
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
  historyQueue = xQueueCreate(100, sizeof(SensorData));

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