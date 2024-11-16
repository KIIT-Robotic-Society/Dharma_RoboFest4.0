#include <SPI.h>
#include <LoRa.h>
#include "FreeRTOS.h"
#include "task.h"
#include <Wire.h>
#include <MPU6050.h>


    volatile char statusMessage[50]; // Size 50 for the string

#define MPU6050_ADDR 0x68 // Default I2C address for MPU6050
#define ACCEL_X 0x3B
#define GYRO_X 0x43

/*
SCK (SPI clock): GP18 (default SPI0 SCK)
MOSI (SPI data output): GP19 (default SPI0 MOSI)
MISO (SPI data input): GP16 (default SPI0 MISO)
CS (chip select): You’ve already assigned GP10 as SS_PIN.
RST (reset pin): You’ve assigned GP9 as RST_PIN.
DIO0 (interrupt): You’ve assigned GP2 as DIO0_PIN.#define SS_PIN 10    // LoRa radio chip select pin*/

#define SS_PIN 10    // LoRa radio chip select pin
#define RST_PIN 9    // LoRa radio reset pin
#define DIO0_PIN 1   // LoRa radio DIO0 pin

MPU6050 mpu;  // Create an instance of the MPU6050 class


// Task handles
TaskHandle_t TaskSender, TaskReceiver;
//Task3;

// Function prototypes
void taskSender(void *pvParameters);
void taskReceiver(void *pvParameters);
//void task3(void *pvParameters);

void setup() {
    Serial.begin(115200);

    // Initialize LoRa
    LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN); // Set CS, RESET, and IRQ pin
    if (!LoRa.begin(915E6)) {                // Initialize LoRa at 915 MHz
        Serial.println("LoRa init failed!");
        while (1);
    }
    Serial.println("LoRa Initialized");

    LoRa.setSignalBandwidth(250E3);  // 250 kHz bandwidth
    LoRa.setSpreadingFactor(7);       // Spreading factor SF7
    LoRa.setCodingRate4(5);           // Coding rate 4/

    // Create sender and receiver tasks
    xTaskCreate(taskSender, "TaskSender", 128, NULL, 1, &TaskSender);
    xTaskCreate(taskReceiver, "TaskReceiver", 128, NULL, 1, &TaskReceiver);

}

void setup1(){
    Serial.println(115200);
    Wire1.setSDA(2); // Set GP0 as SDA
    Wire1.setSCL(3); // Set GP1 as SCL
    Wire1.begin();

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(0x6B); // Power management register
    Wire1.write(0);    // Wake the MPU6050 up
    Wire1.endTransmission(true);
}


void loop() {
    // Empty loop since tasks are running independently
}

void loop1(){
   int16_t ax, ay, az;
int16_t gx, gy, gz;

    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(ACCEL_X); // Starting register for accelerometer data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDR, 6, true); // Request 6 bytes for ax, ay, az
    ax = (Wire1.read() << 8) | Wire1.read();
    ay = (Wire1.read() << 8) | Wire1.read();
    az = (Wire1.read() << 8) | Wire1.read();

    // Read gyroscope data
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(GYRO_X); // Starting register for gyroscope data
    Wire1.endTransmission(false);
    Wire1.requestFrom(MPU6050_ADDR, 6, true); // Request 6 bytes for gx, gy, gz
    gx = (Wire1.read() << 8) | Wire1.read();
    gy = (Wire1.read() << 8) | Wire1.read();
    gz = (Wire1.read() << 8) | Wire1.read();

        gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
    
    Serial.printf("Str %s\n", (char*)statusMessage);
    strcpy((char*)statusMessage, " ");
    if(millis()>=60000){
    Serial.println("RESET");
    software_reset();
}

    delay(500); // Delay 1 second before the next read
}




void taskSender(void *pvParameters) {
    while (1) {
        LoRa.beginPacket();
        LoRa.print("Hello from Sender");
        LoRa.endPacket();
        Serial.println("Message sent");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
    }
}

void taskReceiver(void *pvParameters) {
    while (1) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            String message = "";
            while (LoRa.available()) {
                message += (char)LoRa.read();
            }
            Serial.print("Received: ");
            strcpy((char*)statusMessage, message.c_str());
            Serial.println(message);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 milliseconds
    }
}


void software_reset(){
  watchdog_enable(1,1);
  while(1);
}