
#include "FreeRTOS.h"
#include "task.h"

// Declare two task handles
TaskHandle_t Task1, Task2;

void setup() {
  // Create Task1 for blinking the onboard LED
  xTaskCreate(
    task1,           // Function for Task1
    "Task1",         // Task name (for debugging)
    128,             // Stack size (in words)
    NULL,            // Task input parameter (not used)
    1,               // Task priority
    &Task1           // Task handle
  );

  // Create Task2 for blinking the external LED
  xTaskCreate(
    task2,           // Function for Task2
    "Task2",         // Task name
    128,             // Stack size (in words)
    NULL,            // Task input parameter (not used)
    1,               // Task priority
    &Task2           // Task handle
  ); 
}

void setup1(){
  pinMode(3,OUTPUT);
}

// Task1 function: Blink the onboard LED (GPIO 25)
void task1(void *param) {
  (void) param;
  pinMode(2,OUTPUT);
  while (1) {
    digitalWrite(2, LOW);  // Turn on the onboard LED
    delay(500);
    digitalWrite(2, HIGH);   // Turn off the onboard LED
    delay(500);
  }
}

// Task2 function: Blink an external LED (GPIO 2)
void task2(void *param) {
  (void) param;
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, LOW);  // Turn on the onboard LED
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);   // Turn off the onboard LED
    delay(500);
  }
}

void loop() {
  delay(500);
}

void loop1() {
    digitalWrite(3, LOW);  // Turn on the onboard LED
    delay(500);
    digitalWrite(3, HIGH);
    delay(500); 
}



