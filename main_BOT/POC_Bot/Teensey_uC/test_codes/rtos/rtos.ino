#include <Arduino.h>
#include "TeensyThreads.h"

const int LED_RED = 15;
const int LED_GREEN = 19;
const int PUSH = 23;
int count = 0;

void blinkRed(){
  while (1) {
    digitalWrite(LED_RED, HIGH);
    threads.delay(250);
    digitalWrite(LED_RED, LOW);
    threads.delay(250);
    threads.yield();
  }
}
void blinkGreen() {
  while (1) {
    digitalWrite(LED_GREEN, HIGH);
    threads.delay(500);
    digitalWrite(LED_GREEN, LOW);
    threads.delay(500);
    threads.yield();
  }
}
void push() {
  while (1) {
    if(digitalRead(PUSH) == 0){
      while(digitalRead(PUSH) == 0){
          count++;
          Serial.print("TOCO ");
          Serial.println(count);
          threads.delay(250); // efecto rebote
      }
    }
    threads.yield();
  }
}
void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(PUSH, INPUT_PULLUP);
  threads.addThread(blinkRed);
  threads.addThread(blinkGreen);
  threads.addThread(push);
}

void loop() {
  // nada
}