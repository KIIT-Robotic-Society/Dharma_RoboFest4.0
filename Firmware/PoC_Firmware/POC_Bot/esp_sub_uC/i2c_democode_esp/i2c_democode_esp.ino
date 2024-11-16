#include <Wire.h>

#define SLAVE_ADDR 0x78  // Set I2C address for ESP32-S3 as slave

void requestEvent() {
    String response = "HI";
    byte responseData[response.length() + 1];
    response.toCharArray((char*)responseData, response.length() + 1);
    Wire1.write(responseData, response.length() + 1);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Wire1.begin(SLAVE_ADDR,4,5,400000);
    Wire1.onRequest(requestEvent);
    Serial.println("ESP32 I2C Slave Ready...");
}

// Function called when data is requested by the master

void loop() {
  // Slave logic; here it could be preparing data for the master to request
  delay(1000);  // Adjust delay as needed
}
