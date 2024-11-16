#include <Wire.h>

#define SLAVE_ADDRESS 8  // Address of the I2C slave device

void setup() {
  Wire.begin();          // Start I2C as master
  Serial.begin(115200);  // Start serial communication
  delay(1000);          // Wait for slave to initialize
}

void loop() {
  Wire.beginTransmission(SLAVE_ADDRESS); // Start communication with the slave
  Wire.write("Hello");                    // Send a message to the slave
  Wire.endTransmission();                 // End transmission

  // Request 5 bytes of data from the slave
  Wire.requestFrom(SLAVE_ADDRESS, 5);
  while (Wire.available()) {               // Wait for data to be available
    char c = Wire.read();                  // Read a byte
    Serial.print(c);                       // Print the received character
  }
  Serial.println();                       // New line after receiving data

  delay(1000);                            // Wait before sending next message
}
