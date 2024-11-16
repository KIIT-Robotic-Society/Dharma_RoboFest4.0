// Pin Definition
const int ledPin = 13;  // Pin for the onboard LED

void setup() {
    pinMode(ledPin, OUTPUT);  // Set LED pin as OUTPUT
}

void loop() {
    Serial.println("Helloi");                // Wait for 500 milliseconds (0.5 seconds)
}