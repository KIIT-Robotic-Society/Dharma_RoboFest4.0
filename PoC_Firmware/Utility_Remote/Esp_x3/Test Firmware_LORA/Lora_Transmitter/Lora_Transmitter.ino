#include <SPI.h>
#include <LoRa.h>

#define NSS_PIN   4    // CS
#define RESET_PIN 14    // Reset pin
#define DIO0_PIN  12    // DIO0 pin

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize LoRa
    LoRa.setPins(NSS_PIN, RESET_PIN, DIO0_PIN); 
    if (!LoRa.begin(433E6)) { //frequency ( 433E6 for 433 MHz)
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    Serial.println("LoRa Initializing OK!");
}

void loop() {
    Serial.println("Sending packet...");
    LoRa.beginPacket();
    LoRa.print("Hello, LoRa!");
    LoRa.endPacket();

    delay(2000); 
}
