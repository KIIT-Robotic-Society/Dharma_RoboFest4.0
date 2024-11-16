#include <esp_now.h>
#include <WiFi.h>

// MAC Address of the receiver
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0xE7, 0x1D, 0x74};



// Structure to hold outgoing data
typedef struct struct_message {
  char data[200];  // Adjust size as necessary
} struct_message;

struct_message outgoingData;  // Structure to store the message

// Variable to store incoming data
String incomingReadings;

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
  if (i == 10) {
    // Now, values[0] to values[10] hold your parsed values
    Serial.println("Parsed Data:");
    Serial.println("Pitch: " + values[0]);
    Serial.println("Roll: " + values[1]);
    Serial.println("Yaw: " + values[2]);
    Serial.println("Latitude: " + values[3]);
    Serial.println("Longitude: " + values[4]);
    Serial.println("Satellite Count: " + values[5]);
    Serial.println("BPM: " + values[6]);
    Serial.println("Avg BPM: " + values[7]);
    Serial.println("Dog Mode: " + values[8]);
    Serial.println("Failsafe: " + values[9]);
    Serial.println("ADC Value: " + values[10]);
  } else {
    Serial.println("Invalid data received. Not enough values.");
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Register for a callback when data is sent
  esp_now_register_send_cb(OnDataSent);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);  // Copy the peer address
  peerInfo.channel = 0;  // Channel is default 0
  peerInfo.encrypt = false;  // No encryption

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Create the message to send
  String dataToSend = "1,20,30,21";  // Example data with ADC value
  dataToSend.toCharArray(outgoingData.data, 200);  // Convert the string to a character array and store in outgoingData

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingData, sizeof(outgoingData)); 
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  delay(1000);  // Wait before sending again
}
