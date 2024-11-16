#include <SPI.h>
#include <LoRa.h>

// Pin definitions for LoRa module
#define NSS_PIN   4    // CS
#define RESET_PIN 14   // Reset pin
#define DIO0_PIN  12   // DIO0 pin

// Declare task handles
TaskHandle_t sendTaskHandle;
TaskHandle_t receiveTaskHandle;

// Constants for core assignment
const int sendTaskCore = 0;    // Run sending task on Core 0
const int receiveTaskCore = 1; // Run receiving task on Core 1

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize LoRa
    LoRa.setPins(NSS_PIN, RESET_PIN, DIO0_PIN);
    if (!LoRa.begin(433E6)) { // Frequency (433 MHz)
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    Serial.println("LoRa Initializing OK!");

    // Create tasks for sending and receiving on different cores
  xTaskCreatePinnedToCore(
    sendTask,              // Function to be executed
    "SendTask",            // Name of the task
    10000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &sendTaskHandle,             // Task handle
    0 
  );

   xTaskCreatePinnedToCore(
    receiveTask,              // Function to be executed
     "ReceiveTask",            // Name of the task
    10000,              // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &receiveTaskHandle,             // Task handle
    1 
  );
  
}

// Task to send packets via LoRa
void sendTask(void *pvParameters) {
    while (1) {
        Serial.println("Sending packet from Core 0...");
        LoRa.beginPacket();
        LoRa.print("Hello, LoRa from Core 0!");
        LoRa.endPacket();

        // Delay between sends (2 seconds)
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Task to receive packets via LoRa
void receiveTask(void *pvParameters) {
    while (1) {
            // Start LoRa receive mode
        LoRa.receive();
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            Serial.print("Received packet on Core 1: ");

            // Read the received packet
            while (LoRa.available()) {
                Serial.print((char)LoRa.read());
            }
            Serial.println();
        }

        // Short delay to prevent task starvation
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void loop() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
