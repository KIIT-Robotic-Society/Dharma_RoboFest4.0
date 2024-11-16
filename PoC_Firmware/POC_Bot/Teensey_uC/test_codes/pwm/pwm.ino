#include <ESP32Servo.h>

Servo myServo;    
      // create a Servo object
int servo_pin = 4;      // define the servo pin
int angle = 90;          // variable to store the servo position

void setup() {
  myServo.attach(4);  // attach the servo to the pin
  delay(1000);
  myServo.write(90);
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (angle = 60; angle <= 120; angle += 1) { 
    myServo.write(angle);       // move servo to specified angle
    delay(30);                // delay for smooth movement
}
delay(500);
  for (angle = 120; angle >= 60; angle -= 1) { 
    myServo.write(angle);       // move servo to specified angle
    delay(30);                // delay for smooth movement
}
}
