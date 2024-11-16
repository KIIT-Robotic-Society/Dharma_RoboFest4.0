#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define J1X 4
#define J1Y 5
#define J2X 6
#define J2Y 7
#define J3X 8
#define J3Y 3
#define J4X 9
#define J4Y 10
#define toggle1 37
#define toggle2 36
#define toggle3 35

// Variable declaration
volatile bool mode_Centri = 0;
volatile bool mode_Scout = 0;
volatile bool mode_Follow = 0;

volatile int analogVal_J1X = 0;
volatile int analogVal_J1Y = 0;
volatile int analogVal_J2X = 0;
volatile int analogVal_J2Y = 0;
volatile int analogVal_J3X = 0;
volatile int analogVal_J3Y = 0;
volatile int analogVal_J4X = 0;
volatile int analogVal_J4Y = 0;

const int val = 30;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

#define SLAVE_ADDR 0x08  // I2C address of ESP32 (Slave)

void readADC() {
    for (int i = 0; i <= val; i++) {
        analogVal_J1X = analogRead(J1X) + analogVal_J1X;
        analogVal_J1Y = analogRead(J1Y) + analogVal_J1Y;
        analogVal_J2X = analogRead(J2X) + analogVal_J2X;
        analogVal_J2Y = analogRead(J2Y) + analogVal_J2Y;
        analogVal_J3X = analogRead(J3X) + analogVal_J3X;
        analogVal_J3Y = analogRead(J3Y) + analogVal_J3Y;
        analogVal_J4X = analogRead(J4X) + analogVal_J4X;
        analogVal_J4Y = analogRead(J4Y) + analogVal_J4Y;
    }
}

void avgADCval() {
    analogVal_J1X = analogVal_J1X / val;
    analogVal_J1Y = analogVal_J1Y / val;
    analogVal_J2X = analogVal_J2X / val;
    analogVal_J2Y = analogVal_J2Y / val;
    analogVal_J3X = analogVal_J3X / val;
    analogVal_J3Y = analogVal_J3Y / val;
    analogVal_J4X = analogVal_J4X / val;
    analogVal_J4Y = analogVal_J4Y / val;
}

void readstate() {
    mode_Centri = digitalRead(toggle1);
    mode_Scout = digitalRead(toggle2);
    mode_Follow = digitalRead(toggle3);
}

void task1(void *pvParameters) {
    while (1) {
        readADC();
        avgADCval();
        readstate();
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to prevent watchdog
    }
}

void task2(void *pvParameters) {
    while (1) {
        // Mode selection logic goes here
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}

void task3(void *pvParameters) {
    while (1) {
        // I2C send logic goes here
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}

void task4(void *pvParameters) {
    while (1) {
        Serial.print("Joystick 1 X: ");
        Serial.println(analogVal_J1X);
        Serial.print("Joystick 1 Y: ");
        Serial.println(analogVal_J1Y);
        Serial.print("Joystick 2 X: ");
        Serial.println(analogVal_J2X);
        Serial.print("Joystick 2 Y: ");
        Serial.println(analogVal_J2Y);
        Serial.print("Joystick 3 X: ");
        Serial.println(analogVal_J3X);
        Serial.print("Joystick 3 Y: ");
        Serial.println(analogVal_J3Y);
        Serial.print("Joystick 4 X: ");
        Serial.println(analogVal_J4X);
        Serial.print("Joystick 4 Y: ");
        Serial.println(analogVal_J4Y);
        Serial.print("State Centri: ");
        Serial.println(mode_Centri);
        Serial.print("State Scout: ");
        Serial.println(mode_Scout);
        Serial.print("State Follow: ");
        Serial.println(mode_Follow);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}

void requestEvent() {
    vTaskSuspendAll();
    // I2C request logic goes here
    xTaskResumeAll();
}

void setup() {
    Serial.begin(115200);

    pinMode(J1X, INPUT);
    pinMode(J1Y, INPUT);
    pinMode(J2X, INPUT);
    pinMode(J2Y, INPUT);
    pinMode(J3X, INPUT);
    pinMode(J3Y, INPUT);
    pinMode(J4X, INPUT);
    pinMode(J4Y, INPUT);

    pinMode(toggle1, INPUT_PULLUP);
    pinMode(toggle2, INPUT_PULLUP);
    pinMode(toggle3, INPUT_PULLUP);

    Wire.begin(SLAVE_ADDR);
    Wire.setClock(1000000);
    Wire.onRequest(requestEvent);

    // Task to read ADCs
    xTaskCreatePinnedToCore(task1, "Task1", 10000, NULL, 1, &Task1, 1);
    // Task to select mode
    xTaskCreatePinnedToCore(task2, "Task2", 20000, NULL, 1, &Task2, 0);
    // Task to send data by I2C
    xTaskCreatePinnedToCore(task3, "Task3", 20000, NULL, 2, &Task3, 0);
    // Task to print to Serial
    xTaskCreatePinnedToCore(task4, "Task4", 20000, NULL, 3, &Task4, 1);
}

void loop() {
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Loop delay
}
