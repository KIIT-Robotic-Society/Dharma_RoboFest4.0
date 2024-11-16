#include <Wire.h>

#define SLAVE_ADDRESS 0x78  // I2C address of ESP32-S3

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C, Teensy default SDA=18, SCL=19
  Wire.setClock(400000);
  Serial.println("Teensy 4.1 I2C master initialized.");
}

void loop() {
  // Request data from the ESP32-S3 slave
  Wire.requestFrom(SLAVE_ADDRESS, 78); // Request 70 bytes from ESP32-S3

  // Read and process the data received from the slave
  if (Wire.available()) {
    String receivedMessage = "";
    while (Wire.available()) {
      char c = Wire.read();
      receivedMessage += c;
    }
    Serial.print("Received from ESP32-S3: ");
    Serial.println(receivedMessage);

    // Parse values between "x" and "y"
    int startIndex = receivedMessage.indexOf("x,") + 2; // Position after "x,"
    int endIndex = receivedMessage.indexOf(",y");       // Position before ",y"
    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
      String dataSection = receivedMessage.substring(startIndex, endIndex);
      
      // Split dataSection by commas
      int index = 0;
      String values[12];  // Array to hold each parsed value
      while (dataSection.length() > 0 && index < 12) {
        int commaIndex = dataSection.indexOf(',');
        if (commaIndex == -1) {
          values[index++] = dataSection;
          break;
        }
        values[index++] = dataSection.substring(0, commaIndex);
        dataSection = dataSection.substring(commaIndex + 1);
      }

      // Display each parsed value with labels
      if (index >= 12) {  // Check if all values were parsed
        Serial.println("Parsed Values:");
        Serial.println("aX: " + values[0]);
        Serial.println("aY: " + values[1]);
        Serial.println("aZ: " + values[2]);
        Serial.println("gX: " + values[3]);
        Serial.println("gY: " + values[4]);
        Serial.println("gZ: " + values[5]);
        Serial.println("Latitude: " + values[6]);
        Serial.println("Longitude: " + values[7]);
        Serial.println("Distance 1: " + values[8]);
        Serial.println("Distance 2: " + values[9]);
        Serial.println("Distance 3: " + values[10]);
        Serial.println("Distance 4: " + values[11]);
      } else {
        Serial.println("Incomplete data received.");
      }
    } else {
      Serial.println("Data markers 'x' or 'y' not found.");
    }
  }

  delay(10);  // Delay between data requests
}
