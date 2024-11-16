#include <SPI.h>
#include <LoRa.h>

/*SCK (Clock) is on pin 13.
MISO (Master In Slave Out) is on pin 12. RX
MOSI (Master Out Slave In) is on pin 11.
CS (Chip Select, also called SS) can be any available pin, so you can keep using pin 10 as in your current code.
DIO0 and RST can also be assigned to any available digital pins, which in your current setup are pin 2 and pin 9 respectively.*/

// Pin assignments for Arduino Uno
#define SS_PIN 10    // LoRa radio chip select pin (can be any digital pin)
#define RST_PIN 9    // LoRa radio reset pin
#define DIO0_PIN 2   // LoRa radio DIO0 pin

void setup() {
    Serial.begin(115200);

    // Initialize LoRa
    LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN); // Set CS, RESET, and IRQ pin
    if (!LoRa.begin(915E6)) {                // Initialize LoRa at 915 MHz
        Serial.println("LoRa init failed!");
        while (1);  // Stop execution if LoRa initialization fails
    }
    Serial.println("LoRa Initialized");

    LoRa.setSignalBandwidth(250E3);  // 250 kHz bandwidth
    LoRa.setSpreadingFactor(7);       // Spreading factor SF7
    LoRa.setCodingRate4(5);           // Coding rate 4/
}

void loop() {
    // Handle sending data every 1 second
    sendMessage();
    // Handle receiving data if available
    receiveMessage();

    //Serial.println(millis());

    // Small delay to allow for other operations (optional)
    delay(10);

}

void sendMessage() {
    static unsigned long lastSendTime = 0;
    unsigned long currentTime = millis();

    // Send message every 1 second
    if (currentTime - lastSendTime > 1000) {
        LoRa.beginPacket();
        LoRa.print("Hello, !");
        LoRa.endPacket();
        Serial.println("Message sent");
        lastSendTime = currentTime;
    }
}

void receiveMessage() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        Serial.print("Received: ");
        Serial.println(message);
    }
}
