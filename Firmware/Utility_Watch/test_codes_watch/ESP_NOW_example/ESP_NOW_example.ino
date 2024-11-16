#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MAX_DATA_LENGTH 100

// Variables to hold the broadcast address and incoming data
uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x27, 0xD3, 0x10}; // Replace with actual peer's MAC address

// For sending data
const char* success;
esp_now_peer_info_t peerInfo;
// For receiving data
String receivedData;

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS){
    success = "Delivery Success :)";
  } else {
    success = "Delivery Fail :(";
  }
}

// Updated onDataReceived with esp_now_recv_info parameter
void onDataReceived(const esp_now_recv_info *info, const uint8_t *data, int len) {
    receivedData = "";
    receivedData.reserve(len + 1);
    receivedData += (const char *)data; // Create a string from received data

    // Print received data for debugging
    Serial.print("Received data: ");
    Serial.println(receivedData);

    // Parse the incoming string
    int mode, remoteBatteryPercentage, pocBatteryPercentage;
    bool emeryDetected;

    // Split the string by comma
    char *token = strtok(const_cast<char *>(receivedData.c_str()), ",");
    if (token != NULL) {
        mode = atoi(token); // Convert mode to int
        token = strtok(NULL, ",");
    }
    if (token != NULL) {
        remoteBatteryPercentage = atoi(token); // Convert remote battery to int
        token = strtok(NULL, ",");
    }
    if (token != NULL) {
        pocBatteryPercentage = atoi(token); // Convert POC battery to int
        token = strtok(NULL, ",");
    }
    if (token != NULL) {
        emeryDetected = (strcmp(token, "1") == 0); // Check if emeryDetected is "1" (true)
    }

    // Print parsed data
    Serial.print("Mode: "); Serial.println(mode);
    Serial.print("Remote Battery: "); Serial.println(remoteBatteryPercentage);
    Serial.print("POC Battery: "); Serial.println(pocBatteryPercentage);
    Serial.print("Emery Detected: "); Serial.println(emeryDetected ? "Yes" : "No");
}

// Task to send data every second
void taskSendData(void *pvParameters) {
  String sent = "";
  while (true) {
    // Prepare the data string
    sent = String("-0.342") + "," +
           String("-9.83") + "," +
           String("0.34") + "," +
           String("23.23") + "," +
           String("82.34") + "," +
           String("3") + "," +
           String("30") + "," +
           String("20") + "," +
           String("2") + "," + 
           String("345");

    // Convert the String to a C-string for sending
    const char* message = sent.c_str();

    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)message, strlen(message)); 

    if (result == ESP_OK) {
      Serial.println("Data sent successfully: " + sent);
    } else {
      Serial.println("Error sending data");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Wi-Fi in station mode
  WiFi.mode(WIFI_STA);
  
  // Set up ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Add peer for ESP-NOW
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  // Use the current channel
  peerInfo.encrypt = false;  // No encryption

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register the sending and receiving callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(onDataReceived); // Updated to new function signature

  // Create the sending task
  xTaskCreate(taskSendData, "Send Task", 2048, NULL, 1, NULL);
}

void loop() {
  // Main loop can remain empty, as tasks handle the functionality
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
