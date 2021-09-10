#include <Arduino.h>

bool state = 0;

void toggle(){
    state = !state;
    digitalWrite(PA9, state);
}

void setup(){

    pinMode(PC14, INPUT_PULLUP);
    pinMode(PA9, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(PC14), toggle, CHANGE);
}

void loop(){}