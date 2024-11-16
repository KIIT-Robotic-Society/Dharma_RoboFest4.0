#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SLAVE_ADDR 0x08  // I2C address of ESP32 (Slave)
SemaphoreHandle_t i2cSemaphore;

String receivedData = "";

#define QUEUE_SIZE 5
QueueHandle_t dataQueue;
String sendData;

uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0xE7, 0x1D, 0x74};

volatile int mode = 4;
volatile int pocbattery = 56;
volatile int remotebattery = 100;
volatile int enemydetected = 1;

typedef struct struct_message {
  char data[200]; 
} struct_message;

typedef struct message {
    int Pitch;
    int Roll;
    int Yaw;
    int lat;
    int lng;
    int satellite;
    int BPM;
    int AvgBPM;
    int Mode;
    int FailSafe;
    int Battery;
} message;

message espNowData;

struct_message outgoingData;

// Variable to store incoming data
String incomingReadings;

void receiveEvent(int bytes) {
  receivedData = "";  // Clear the buffer before receiving new data

  // Read all the available bytes and append them to the buffer
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }

  Serial.print("Received: ");
  Serial.println(receivedData);  // Print the entire received string to Serial Monitor

  // Now parse the received string in the format: (mode, poc_bot_battery, remote_battery, enemy_detected)
  parseReceivedData_I2C();
}

void parseReceivedData_I2C() {
  // Remove the parentheses from the string
  receivedData = receivedData.substring(1, receivedData.length() - 1); // Remove first '(' and last ')'
  
  // Split the string by commas and store each part
  int firstComma = receivedData.indexOf(',');
  int secondComma = receivedData.indexOf(',', firstComma + 1);
  int thirdComma = receivedData.indexOf(',', secondComma + 1);

  if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
    // Extract mode, poc_bot_battery, remote_battery, and enemy_detected
    String modeStr = receivedData.substring(0, firstComma);               // First part -> mode
    String pocBotBatteryStr = receivedData.substring(firstComma + 1, secondComma); // Second part -> poc_bot_battery
    String remoteBatteryStr = receivedData.substring(secondComma + 1, thirdComma); // Third part -> remote_battery
    String enemyDetectedStr = receivedData.substring(thirdComma + 1);    // Fourth part -> enemy_detected

    // Convert the strings to integers
    mode = modeStr.toInt();
    pocbattery = pocBotBatteryStr.toInt();
    remotebattery = remoteBatteryStr.toInt();
    enemydetected = enemyDetectedStr.toInt();
  }
}

void requestEvent() {
  String response = String(espNowData.Pitch) + "," + String(espNowData.Roll) + "," + 
                    String(espNowData.Yaw) + "," + String(espNowData.lat) + "," + 
                    String(espNowData.lng) + "," + String(espNowData.satellite) + "," + 
                    String(espNowData.BPM) + "," + String(espNowData.AvgBPM) + "," +
                    String(espNowData.Mode) + "," + String(espNowData.FailSafe) + "," + 
                    String(espNowData.Battery); 
  byte responseData[response.length() + 1];

  const char* responseChar = response.c_str();  // Convert String to const char*
  Wire.write((uint8_t*)responseChar, strlen(responseChar));  // Send data as byte array
  Serial.print("Sent");
  Serial.println(response);  // Print the response for debugging
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  char incomingString[len + 1];
  memcpy(incomingString, incomingData, len);
  incomingString[len] = '\0';  // Null-terminate the received data

  incomingReadings = String(incomingString);  // Convert to String

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println("Received data: " + incomingReadings);

  // Parse the incoming data
  parseIncomingData(incomingReadings);
}

// Function to parse the incoming data
void parseIncomingData(String data) {
  int commaIndex = 0;
  String values[11];  // Array to store the parsed values (increased to 11 for ADC value)
  int i = 0;

  // Parse string by commas
  while ((commaIndex = data.indexOf(',')) != -1 && i < 10) {
    values[i] = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);  // Update string to remove the parsed value
    i++;
  }
  values[i] = data;  // Store the last value

  // Ensure there are at least 11 values before proceeding
  if (i == 10) { // We have exactly 10 parsed values
    // Now, values[0] to values[9] hold your parsed values
    espNowData.Pitch = values[0].toInt();
    espNowData.Roll = values[1].toInt();
    espNowData.Yaw = values[2].toInt();
    espNowData.lat = values[3].toInt();
    espNowData.lng = values[4].toInt();
    espNowData.satellite = values[5].toInt();
    espNowData.BPM = values[6].toInt();
    espNowData.AvgBPM = values[7].toInt();
    espNowData.Mode = values[8].toInt();
    mode = values[8].toInt(); 
    espNowData.FailSafe = values[9].toInt();
    espNowData.Battery = values[10].toInt();

  } else {
    Serial.println("Invalid data received. Not enough values.");
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

// Sending data task
void task1(void *pvParameters) {
  while (true) {
        String preparedData = String(mode) + "," +
                              String(pocbattery) + "," +
                              String(remotebattery) + "," +
                              String(enemydetected);

        // Send the data to the queue
        if (xQueueSend(dataQueue, &preparedData, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to send data to the queue");
        } else {
            Serial.println("Data queued successfully: " + preparedData);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Task runs every 100 ms
    }
}

void task2(void *pvParameters) {
    String queueData;

    while (true) {
        if (xQueueReceive(dataQueue, &queueData, portMAX_DELAY) == pdPASS) {
            String preparedData = queueData;

            // Convert String to char* for esp_now_send
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)preparedData.c_str(), preparedData.length());

            if (result == ESP_OK) {
                Serial.println("Data sent successfully: " + preparedData);
            } else {
                Serial.println("Error sending data: " + String(result));
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Task runs every 100 ms
    }
}

// Sending data via I2C task
void task3(void *pvParameters) {
    while (true) {
        // Lock the semaphore for I2C access
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
            // Send data via I2C if available
            vTaskDelay(10); // Allow a short delay before sending
            requestEvent();
            xSemaphoreGive(i2cSemaphore);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Task runs every 500 ms
    }
}

// Receiving data via I2C task
void task4(void *pvParameters) {
    while (true) {
        // Lock the semaphore for I2C access
        if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
            // Read data from I2C
            Wire.requestFrom(SLAVE_ADDR, 32);  // Request 32 bytes from I2C slave
            while (Wire.available()) {
                char c = Wire.read();
                receivedData += c; // Append received character
            }
            if (receivedData.length() > 0) {
                parseReceivedData_I2C(); // Parse received data
                receivedData = ""; // Clear the buffer
            }
            xSemaphoreGive(i2cSemaphore);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Task runs every 500 ms
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(requestEvent);  // Register event handler for sending data
    Wire.onReceive(receiveEvent);  // Register event handler for receiving data

    WiFi.mode(WIFI_STA);  // Setup ESP32 as a WiFi station
    esp_now_init();
    esp_now_register_recv_cb(OnDataRecv); // Register the callback for data received
    esp_now_register_send_cb(OnDataSent); // Register the callback for data sent

    // Initialize queue for communication between tasks
    dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(String));

    // Initialize the semaphore for I2C access
    i2cSemaphore = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreate(task1, "Task1", 2048, NULL, 1, &Task1);
    xTaskCreate(task2, "Task2", 2048, NULL, 2, &Task2);
    xTaskCreate(task3, "Task3", 2048, NULL, 1, &Task3);
    xTaskCreate(task4, "Task4", 2048, NULL, 1, &Task4);
}

void loop() {
    // Main loop can be empty because tasks handle the logic
}
