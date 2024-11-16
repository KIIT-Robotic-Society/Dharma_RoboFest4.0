#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define SLAVE_ADDR 0x32  // I2C address of ESP32 (as Slave)

// Pin definitions for LoRa module
#define NSS_PIN   4    // CS
#define RESET_PIN 14   // Reset pin
#define DIO0_PIN  12   // DIO0 pin

String receivedData_rasp = ""; // Receiving String for Raspberry Pi I2C

// Declare task handles
TaskHandle_t sendTaskHandle;

// Semaphore to protect shared resource
SemaphoreHandle_t xSemaphore;


void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize I2C communication with Raspberry Pi
    Wire.begin(SLAVE_ADDR);  // Set ESP32 as I2C Slave
    Wire.onReceive(receiveEvent);  // Register receive event

    Serial.println("ESP32 I2C Slave Ready...");

    // Initialize LoRa
    LoRa.setPins(NSS_PIN, RESET_PIN, DIO0_PIN);
    if (!LoRa.begin(433E6)) { // Frequency (433 MHz)
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    
    Serial.println("LoRa Initializing OK!");

    // Create semaphore
    xSemaphore = xSemaphoreCreateMutex();

    // Create tasks for sending and receiving on different cores
    xTaskCreatePinnedToCore(sendTask, "SendTask", 10000, NULL, 1, &sendTaskHandle, 0);
}

void sendTask(void *pvParameters) {
    while (1) {
        LoRa.beginPacket();
        LoRa.print(receivedData_rasp);
        Serial.println("Sent packet: " + receivedData_rasp);
        LoRa.endPacket();

        vTaskDelay(2000 / portTICK_PERIOD_MS); // 2-second delay
    }
}

void receiveEvent(int howMany) {
    // Take the semaphore
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        String tempData = "";  // Create a temporary string to store received data
        while (Wire.available()) {
            char c = Wire.read();
            tempData += c;
        }
        receivedData_rasp = tempData;  // Copy the data to the global variable
        Serial.println("Received:");
        Serial.println(receivedData_rasp);
        // Give the semaphore back
        xSemaphoreGive(xSemaphore);
    }
}



void loop() {
    // Empty loop; tasks are handled in FreeRTOS
}
