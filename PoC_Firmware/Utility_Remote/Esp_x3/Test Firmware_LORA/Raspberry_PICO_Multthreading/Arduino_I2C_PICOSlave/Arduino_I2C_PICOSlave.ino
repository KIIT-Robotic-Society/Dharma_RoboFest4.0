#include <Wire.h>

#define SLAVE_ADDRESS 8  // I2C slave address

void setup() {
  Wire.begin(SLAVE_ADDRESS);             // Start I2C as slave
  Wire.onReceive(receiveEvent);          // Register receive event handler
  Wire.onRequest(requestEvent);          // Register request event handler
  Serial.begin(115200);                   // Start serial communication
}

void loop() {
  // Main loop does nothing, everything is handled in interrupts
}

// Function to handle data received from the master
void receiveEvent(int howMany) {
  while (Wire.available()) {              // Loop through all received bytes
    char c = Wire.read();                 // Read each byte
    Serial.print(c);                      // Print received character
  }
  Serial.println();                       // New line after receiving data
}

// Function to handle requests from the master
void requestEvent() {
  Wire.write("Pico");                     // Send response to master
}
