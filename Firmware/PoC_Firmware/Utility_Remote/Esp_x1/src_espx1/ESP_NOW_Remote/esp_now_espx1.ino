/*
--------------------------xxxxxxxxxxxxxxxxxx--------------------------

Task 1: This Task handles the ESP_NOW Queue
Queue the data to be sent by ESP_NOW

--------------------------xxxxxxxxxxxxxxxxxx--------------------------
Task 2: This handles the ESP_NOW sending
Sents the data through ESP_NOW


--------------------------xxxxxxxxxxxxxxxxxx--------------------------
Task 3: This task print the serial monitor

--------------------------xxxxxxxxxxxxxxxxxx--------------------------

**PINS***

--I2C--
Gnd->Gnd
GPIO 6(SDA)->GPIO 2(SCL)
GPIO 7(SCL)->GPIO 3(SCL)

*/

//The mac address of the receiver ESP is 64:e8:33:7f:81:a0
//and this esp is 64:e8:33:7f:83:8c





#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SLAVE_ADDR 0x78  // I2C address of ESP32 (Slave)
SemaphoreHandle_t i2cSemaphore;


String receivedData = ""; //receiving String

#define QUEUE_SIZE 5
QueueHandle_t dataQueue;
String sendData;

uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0xE7, 0x1D, 0x74};  //single peer


volatile int Mode_raspberry;
volatile int Mode_esp;
volatile int pocbattery;
volatile int remotebattery;
volatile int enemydetected;

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
    int FailSafe;
    int Battery;
} message;

message espNowData;

struct_message outgoingData;


String incomingReadings; //ncoming data

//--------------------------ESP-NOW--------------------------

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  char incomingString[len + 1];
  memcpy(incomingString, incomingData, len);
  incomingString[len] = '\0';  // Null-terminate

  incomingReadings = String(incomingString);  // Convert to String

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println("Received data: " + incomingReadings);

  parseIncomingData(incomingReadings); //Parse the data
}

void parseIncomingData(String data) {
  int commaIndex = 0;
  String values[11];  // Array to store the parsed values
  int i = 0;

  // Parse string by commas
  while ((commaIndex = data.indexOf(',')) != -1 && i < 10) {
    values[i] = data.substring(0, commaIndex);
    data = data.substring(commaIndex + 1);  // Update string to remove the parsed value
    i++;
  }
  values[i] = data;  // Storing the last value

  if (i == 10) { 
    espNowData.Pitch = values[0].toInt();
    espNowData.Roll = values[1].toInt();
    espNowData.Yaw = values[2].toInt();
    espNowData.lat = values[3].toInt();
    espNowData.lng = values[4].toInt();
    espNowData.satellite = values[5].toInt();
    espNowData.BPM = values[6].toInt();
    espNowData.AvgBPM = values[7].toInt();
    Mode_esp = values[8].toInt();
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
//--------------------------xxxxxxxxxxxxxxxxxx--------------------------


//--------------------------Free-RTOS-----------------------------------
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

//queuing data thorugh ESP-NOW
void task1(void *pvParameters) {
   while (true) {
        String preparedData = String(Mode_raspberry) + "," +
                              String(pocbattery) + "," +
                              String(remotebattery) + "," +
                              String(enemydetected);

        // Send the data to the queue
        if (xQueueSend(dataQueue, &preparedData, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to send data to the queue");
        } else {
            Serial.println("Data queued successfully: " + preparedData);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//sending the data through ESP-NOW
void task2(void *pvParameters) {
    String queueData;

    while (true) {
        if (xQueueReceive(dataQueue, &queueData, portMAX_DELAY) == pdPASS) {
            String preparedData = queueData;

            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)preparedData.c_str(), preparedData.length());

            if (result == ESP_OK) {
                Serial.println("Data sent successfully: " + preparedData);
            } else {
                Serial.println("Error sending data: " + String(result));
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}

//print the data for debugging
void task3(void *pvParameters) {
    while (true) {
        /*Serial.println("Parsed Data:");
        Serial.println("Pitch: "+ String(espNowData.Pitch));
        Serial.println("Roll: " + String(espNowData.Roll));
        Serial.println("Yaw: " + String(espNowData.Yaw));
        Serial.println("Latitude: " + String(espNowData.lat));
        Serial.println("Longitude: " + String(espNowData.lng));
        Serial.println("Satellite Count: " + String(espNowData.satellite));
        Serial.println("BPM: " + String(espNowData.BPM));
        Serial.println("Avg BPM: " + String(espNowData.AvgBPM));
        Serial.println("Dog Mode: " + String(Mode_esp));
        Serial.println(" Mode: " + String(Mode_raspberry));
        Serial.println("Failsafe: " + String(espNowData.FailSafe));
        Serial.println("ADC Value: " + String(espNowData.Battery));

        Serial.print("Mode: "); Serial.println(Mode_raspberry);
        Serial.print("Poc Bot Battery: "); Serial.println(pocbattery);
        Serial.print("Remote Battery: "); Serial.println(remotebattery);
        Serial.print("Enemy Detected: "); Serial.println(enemydetected);*/
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Task runs every 1 second
    }
}
//--------------------------xxxxxxxxxxxxxxxxxx--------------------------

//--------------------------I2C-Handle--------------------------
void requestEvent() {
  vTaskSuspendAll();

    String response = String(espNowData.Pitch) + "," + String(espNowData.Roll) + "," + String(espNowData.Yaw) + "," + 
                      String(espNowData.lat) + "," + String(espNowData.lng) + "," + String(espNowData.satellite) + "," + 
                      String(espNowData.BPM) + "," + String(espNowData.AvgBPM) + "," + String(Mode_esp) + "," + 
                      String(espNowData.FailSafe) + "," + String(espNowData.Battery)+",";

    byte responseData[response.length() + 1];
    response.toCharArray((char*)responseData, response.length() + 1);
    Wire.write(responseData, response.length() + 1);
    xTaskResumeAll();
}


void receiveEvent(int numBytes) {
  unsigned long lastReceiveTime = millis();  // time of the last received data
  const unsigned long timeoutPeriod = 1000;

      receivedData = "";

      while (Wire.available()) {
        char c = Wire.read();
        receivedData += c;
      }

      Serial.print("Received: ");
      Serial.println(receivedData);

      parseReceivedData();
      
      lastReceiveTime = millis(); 
      if (millis() - lastReceiveTime > timeoutPeriod) {
        Serial.println("Error: No data received for a while!");
        lastReceiveTime = millis();  
      } else {
        Serial.println("No data available");
      }
      Serial.println("Emeny ");
      Serial.println(enemydetected);
    }

void parseReceivedData() {

  receivedData = receivedData.substring(1, receivedData.length() - 1); // Remove first '(' and last ')'
  
  int firstComma = receivedData.indexOf(',');
  int secondComma = receivedData.indexOf(',', firstComma + 1);
  int thirdComma = receivedData.indexOf(',', secondComma + 1);
  yield();

  if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
    // Extract mode, poc_bot_battery, remote_battery, and enemy_detected
    String modeStr = receivedData.substring(0, firstComma);               // First part -> mode
    String pocBotBatteryStr = receivedData.substring(firstComma + 1, secondComma); // Second part -> poc_bot_battery
    String remoteBatteryStr = receivedData.substring(secondComma + 1, thirdComma); // Third part -> remote_battery
    String enemyDetectedStr = receivedData.substring(thirdComma + 1);    // Fourth part -> enemy_detected

    // Convert the strings to integers
    Mode_raspberry= modeStr.toInt();
    pocbattery = pocBotBatteryStr.toInt();
    remotebattery = remoteBatteryStr.toInt();
    enemydetected = enemyDetectedStr.toInt();
    yield();
  }
}

void setup() {
  Serial.begin(115200);

  //FOR ESP-S3 XIA0
 // Wire.begin(SLAVE_ADDR);
  //Wire.setClock(1000000);

  //For S3 dev board
  Wire.begin(SLAVE_ADDR,6,7,1000000);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6); 
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  dataQueue = xQueueCreate(QUEUE_SIZE, sizeof(String));
  if (dataQueue == NULL) {
      Serial.println("Error creating queue");
      return;
  }

  i2cSemaphore = xSemaphoreCreateMutex();
  
  if (i2cSemaphore == NULL) {
    Serial.println("Failed to create I2C semaphore");
  }
    
  // Sending Data
  xTaskCreatePinnedToCore(
    task1,              // Function to be executed
    "Task1",            // Name of the task
    20000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &Task1,             // Task handle
    tskNO_AFFINITY      // Core number
  );
    
  // Queue Scheduling
  xTaskCreatePinnedToCore(
    task2,              // Function to be executed
    "Task2",            // Name of the task
    20000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &Task2,             // Task handle
    tskNO_AFFINITY      // Core number
  );

  //Printing data to serial
  xTaskCreatePinnedToCore(
    task3,              // Function to be executed
    "Task3",            // Name of the task
    4096,               // Stack size
    NULL,               // Parameters
    3,                  // Priority
    &Task3,             // Task handle
    tskNO_AFFINITY      // Core number
  );

}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Just to keep the loop alive
}
