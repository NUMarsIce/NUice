/**
 * 
 * Digitizes two AD8495 themocouple amplifiers
 * 
 * */
#include <Arduino.h>
#include <AD8495.h>

AD8495 heat1_therm(A0);
AD8495 heat2_therm(A1);

void setup(){
    Serial.begin(9600);
}

void loop(){
    delay(50); //20 Hz
    Serial.print("0");
    Serial.println((int)(heat1_therm.read()));
    Serial.print("1");
    Serial.println((int)(heat2_therm.read()));
    Serial.print("2");
    Serial.println(analogRead(A3));
}