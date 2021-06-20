/**
 * 
 * Digitizes two AD8495 themocouple amplifiers and a potentiometer
 * 
 * */
#include <Arduino.h>
#include <AD8495.h>

AD8495 heat1_therm(A0);
AD8495 heat2_therm(A1);

void sendData(int data, uint8_t idx){
    Serial.println(data*10+idx%10);
}

void setup(){
    Serial.begin(9600);
}

void loop(){
    delay(50); //20 Hz
    sendData(heat1_therm.read(), 0);
    sendData(heat2_therm.read(), 1);
    sendData(analogRead(A2), 2);
}


