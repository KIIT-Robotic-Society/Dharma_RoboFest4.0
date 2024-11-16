
#include "FreeRTOS.h"
#include "task.h"

// Declare two task handles
TaskHandle_t Task1, Task2;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  

}

void setup1(){
  pinMode(3,OUTPUT);

    // Set up GPIO pins for LEDs
  //pinMode(25, OUTPUT); // Built-in LED

  
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
  taskDebug();
  delay(25);
}

void loop1() {
    digitalWrite(3,OUTPUT);
    delay(25); 
}

void taskDebug(){
  int numberOfTasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *pxTaskStatusArray = new TaskStatus_t [numberOfTasks];
  unsigned long runtime;
  numberOfTasks = uxTaskGetSystemState ( pxTaskStatusArray, numberOfTasks, &runtime ) ;
  
  Serial.printf("# Tasks: %d \n", numberOfTasks); 

  for (int i=0; i < numberOfTasks; i++) {
  
  Serial.printf ("ID: %d %-7s", i, pxTaskStatusArray[i].pcTaskName) ;
  int currentState = pxTaskStatusArray[i].eCurrentState;

  switch (currentState) {
  case 0:
  Serial.printf(" Current State running "); 
  break;

  case 1:
  Serial.printf(" Current State ready");
  break;

  case 2:
  Serial.printf(" Current State blocked");
  break;

  case 3:
  Serial.printf(" Current State suspended");
  break;

  case 4:
  Serial.printf(" Current State deleted");
  break;
}

  Serial.printf(" Priority %-2d", pxTaskStatusArray[i].uxCurrentPriority);
  Serial.printf(" Freestack %-3d", pxTaskStatusArray[i].usStackHighWaterMark);
  Serial.printf(" Runtime %-9d\n", pxTaskStatusArray[i].ulRunTimeCounter);
  }

  delete[] pxTaskStatusArray;

}


