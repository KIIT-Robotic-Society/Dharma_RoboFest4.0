/*

----------------------------------------------------------------
Task 1->reading all the analog values and updating the values
----------------------------------------------------------------
Task 2->setting the mode according to the values
----------------------------------------------------------------
Task 3->Sending the modes by I2C
----------------------------------------------------------------

Joystick 1
X Axis->GPIO 4: 
Y Axis->GPIO 5


Joystick 2
X Axis->GPIO 6
Y Axis->GPIO 7


Joystick 3
X Axis->GPIO 8
Y Axis->GPIO 3


Joystick 4
X Axis->GPIO 9
Y Axis->GPIO 10

----------------------------------------------------------------
Mode Set

Sentri->37
Scout->36
Follow->35
----------------------------------------------------------------

Highest Limit ADC=4000
Lowest Limit ADC=30

*/

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ADC_HIGH 4000
#define ADC_LOW 30
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
#define LED 41

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


// Global variables to track the previous state of the modes
volatile bool prev_mode_Centri = false;  // Stores the previous state of mode_Centri
volatile bool prev_mode_Follow = false;    // Stores the previous state of mode_Flow
volatile bool prev_mode_Scout = false;   // Stores the previous state of mode_Scout

volatile bool mode_using=false;
volatile bool joystick_mode=false;

volatile bool prev_analogVal_J1X=false;
volatile bool prev_analogVal_J1Y=false;
volatile bool prev_analogVal_J2X=false;
volatile bool prev_analogVal_J2Y=false;
volatile bool prev_analogVal_J3X=false;
volatile bool prev_analogVal_J3Y=false;
volatile bool prev_analogVal_J4X=false;
volatile bool prev_analogVal_J4Y=false;


volatile bool joystick=0;
String mode="NULL";
String joystick_mode_sent="NULL";

const int val = 30;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

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


//reading values
void task1(void *pvParameters) {
    while (1) {

        readADC();
        avgADCval();
        readstate();

        if(joystick_mode_sent=="NULL"){
          digitalWrite(LED,HIGH);
        }
        if(joystick_mode_sent!="NULL"){
          digitalWrite(LED,LOW);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay to prevent watchdog
    }
}

//logic for Dog Mode
void task2(void *pvParameters) {
    while (1) {
     if(mode_Centri == false && prev_mode_Centri== false && mode_using==false ){
        mode="Centri";
        prev_mode_Centri=true;
        mode_using=true;
        }
      if(mode_Centri == true && prev_mode_Centri== true && mode_using==true){
        mode="NULL";
        prev_mode_Centri=false;
        mode_using=false;
      }
      if(mode_Follow == false && prev_mode_Follow== false && mode_using==false ){
        mode="Follow";
        prev_mode_Follow=true;
        mode_using=true;
        }

      if(mode_Follow == true && prev_mode_Follow== true && mode_using==true){
        mode="NULL";
        prev_mode_Follow=false;
        mode_using=false;
      }
      if(mode_Scout == false && prev_mode_Scout== false && mode_using==false ){
        mode="Scout";
        prev_mode_Scout=true;
        mode_using=true;
        }

      if(mode_Scout == true && prev_mode_Scout== true && mode_using==true){
        mode="NULL";
        prev_mode_Scout=false;
        mode_using=false;
      }
      
        vTaskDelay(300 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}

//Joystick mode
void task3(void *pvParameters) {
    while (1) {

      if(mode=="Scout"){
//joystick 1 X axis
        if(analogVal_J1X > ADC_HIGH && prev_analogVal_J1X==false && joystick_mode==false){
          joystick_mode_sent="AForward";
          prev_analogVal_J1X=true;
          joystick_mode=true;
        }
        if(analogVal_J1X < ADC_LOW && prev_analogVal_J1X==false && joystick_mode==false){
          joystick_mode_sent="ABackward";
          prev_analogVal_J1X=true;
          joystick_mode=true;
        }
        if(analogVal_J1X < ADC_HIGH && analogVal_J1X > ADC_LOW  && prev_analogVal_J1X== true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J1X=false;
          joystick_mode=false;
        }
//joystick 1 Y axis
        if(analogVal_J1Y > ADC_HIGH && prev_analogVal_J1Y==false && joystick_mode==false){
          joystick_mode_sent="BForward";
          prev_analogVal_J1Y=true;
          joystick_mode=true;
        }
        if(analogVal_J1Y < ADC_LOW && prev_analogVal_J1Y==false && joystick_mode==false){
          joystick_mode_sent="BBackward";
          prev_analogVal_J1Y=true;
          joystick_mode=true;
        }
        if(analogVal_J1Y < ADC_HIGH && analogVal_J1Y > ADC_LOW  && prev_analogVal_J1Y == true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J1Y=false;
          joystick_mode=false;
        }

//joystick 2 X axis   
        if(analogVal_J2X > ADC_HIGH && prev_analogVal_J2X==false && joystick_mode==false){
          joystick_mode_sent="CForward";
          prev_analogVal_J2X=true;
          joystick_mode=true;
        }
        if(analogVal_J2X < ADC_LOW && prev_analogVal_J2X==false && joystick_mode==false){
          joystick_mode_sent="CBackward";
          prev_analogVal_J2X=true;
          joystick_mode=true;
        }
        if(analogVal_J2X < ADC_HIGH && analogVal_J2X > ADC_LOW  && prev_analogVal_J2X== true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J2X=false;
          joystick_mode=false;
        }

//joystick 2 Y axis   
        if(analogVal_J2Y > ADC_HIGH && prev_analogVal_J2Y==false && joystick_mode==false){
          joystick_mode_sent="DForward";
          prev_analogVal_J2Y=true;
          joystick_mode=true;
        }
        if(analogVal_J2Y < ADC_LOW && prev_analogVal_J2Y==false && joystick_mode==false){
          joystick_mode_sent="DBackward";
          prev_analogVal_J2Y=true;
          joystick_mode=true;
        }
        if(analogVal_J2Y < ADC_HIGH && analogVal_J2Y > ADC_LOW  && prev_analogVal_J2Y == true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J2Y=false;
          joystick_mode=false;
        }

//joystick 3 X axis   
        if(analogVal_J3X > ADC_HIGH && prev_analogVal_J3X==false && joystick_mode==false){
          joystick_mode_sent="EForward";
          prev_analogVal_J3X=true;
          joystick_mode=true;
        }
        if(analogVal_J3X < ADC_LOW && prev_analogVal_J3X==false && joystick_mode==false){
          joystick_mode_sent="EBackward";
          prev_analogVal_J3X=true;
          joystick_mode=true;
        }
        if(analogVal_J3X < ADC_HIGH && analogVal_J3X > ADC_LOW  && prev_analogVal_J3X== true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J3X=false;
          joystick_mode=false;
        }

//joystick 3 Y axis   
        if(analogVal_J3Y > ADC_HIGH && prev_analogVal_J3Y==false && joystick_mode==false){
          joystick_mode_sent="FForward";
          prev_analogVal_J3Y=true;
          joystick_mode=true;
        }
        if(analogVal_J3Y < ADC_LOW && prev_analogVal_J3Y==false && joystick_mode==false){
          joystick_mode_sent="FBackward";
          prev_analogVal_J3Y=true;
          joystick_mode=true;
        }
        if(analogVal_J3Y < ADC_HIGH && analogVal_J3Y > ADC_LOW  && prev_analogVal_J3Y == true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J3Y=false;
          joystick_mode=false;
        }


//joystick 4 X axis   
        if(analogVal_J4X > ADC_HIGH && prev_analogVal_J4X==false && joystick_mode==false){
          joystick_mode_sent="GForward";
          prev_analogVal_J4X=true;
          joystick_mode=true;
        }
        if(analogVal_J4X < ADC_LOW && prev_analogVal_J4X==false && joystick_mode==false){
          joystick_mode_sent="GBackward";
          prev_analogVal_J4X=true;
          joystick_mode=true;
        }
        if(analogVal_J4X < ADC_HIGH && analogVal_J4X > ADC_LOW  && prev_analogVal_J4X== true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J4X=false;
          joystick_mode=false;
        }

//joystick 3 Y axis   
        if(analogVal_J4Y > ADC_HIGH && prev_analogVal_J4Y==false && joystick_mode==false){
          joystick_mode_sent="HForward";
          prev_analogVal_J4Y=true;
          joystick_mode=true;
        }
        if(analogVal_J4Y < ADC_LOW && prev_analogVal_J4Y==false && joystick_mode==false){
          joystick_mode_sent="HBackward";
          prev_analogVal_J4Y=true;
          joystick_mode=true;
        }
        if(analogVal_J4Y < ADC_HIGH && analogVal_J4Y > ADC_LOW  && prev_analogVal_J4Y == true && joystick_mode==true){
          joystick_mode_sent="NULL";
          prev_analogVal_J4Y=false;
          joystick_mode=false;
        }

      }

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}

//Premptative
void task4(void *pvParameters) {
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}
//Serial print values
void task5(void *pvParameters) {
    while (1) {
        /*(Serial.print("Joystick 1 X: ");
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
        Serial.println(mode_Follow);*/
        Serial.print("MODE: ");
        Serial.println(mode);
        Serial.println("\n");
        Serial.print("Joystick MODE: ");
        Serial.println(joystick_mode_sent);
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Add a delay to prevent watchdog
    }
}


void requestEvent() {
  vTaskSuspendAll();

    String response = mode + "," + joystick_mode_sent;
    byte responseData[response.length() + 1];
    response.toCharArray((char*)responseData, response.length() + 1);
    Wire.write(responseData, response.length() + 1);

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
    pinMode(LED,OUTPUT);

    pinMode(toggle1, INPUT_PULLUP);
    pinMode(toggle2, INPUT_PULLUP);
    pinMode(toggle3, INPUT_PULLUP);

    Wire.begin(SLAVE_ADDR,11,12,1000000);
    Wire.onRequest(requestEvent);

    // Task to read ADCs
    xTaskCreatePinnedToCore(task1, "Task1", 10000, NULL, 1, &Task1, 1);
    // Task to select mode
    xTaskCreatePinnedToCore(task2, "Task2", 4096, NULL, 2, &Task2, 0);
    //joystick mode
    xTaskCreatePinnedToCore(task3, "Task3", 4096, NULL, 1, &Task3, 0);
    // Task to send data by I2C
    xTaskCreatePinnedToCore(task4, "Task4", 1024, NULL, 4, &Task4, 0);
    // Task to print to Serial
    xTaskCreatePinnedToCore(task5, "Task5", 4096, NULL, 3, &Task5, 1);
}

void loop() {
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Loop delay
}
