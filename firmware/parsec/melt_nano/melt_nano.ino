/**
 * 
 * Digitizes two AD8495 themocouple amplifiers and a potentiometer
 * 
 * */
#include <Arduino.h>
#include <AD8495.h>
#include <NU_Serial.h>

#define RATE 10 //Hz

AD8495 heat1_therm(A0);
AD8495 heat2_therm(A1);

NUSerial ser;

void setup(){
    ser.begin();
}

void loop(){
    delay(1000/RATE-2);
    ser.send(0, (int)heat1_therm.read());
    delay(1);
    ser.send(1, (int)heat2_therm.read());
    delay(1);
    ser.send(2, analogRead(A2));
}


