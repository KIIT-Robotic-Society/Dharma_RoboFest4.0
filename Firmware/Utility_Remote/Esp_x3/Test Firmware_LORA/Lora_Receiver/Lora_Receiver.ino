
#include <SPI.h>
#include <LoRa.h>

#define NSS_PIN   4    // CS
#define RESET_PIN 14   // Reset pin 
#define DIO0_PIN  12   // DIO0 pin

void setup() {
    pinMode(2,OUTPUT);
    Serial.begin(115200);
    while (!Serial);

    LoRa.setPins(NSS_PIN, RESET_PIN, DIO0_PIN); 
    if (!LoRa.begin(433E6)) { //frequency (433E6 for 433 MHz)
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    Serial.println("LoRa Initializing OK!");
    LoRa.receive();
}

void loop() {
    // Check if a packet is available
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("Received packet: ");

        // Read the packet
        while (LoRa.available()) {
            Serial.print((char)LoRa.read());
            digitalWrite(2,HIGH);
            delay(50);
            digitalWrite(2,LOW);
            delay(50);
        }
        Serial.println();
       
        
        LoRa.receive();
    }
}
