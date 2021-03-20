#include <Arduino.h>
#include <AD8495.h>

#define THERM_PIN A0
#define HEAT_PIN 13


AD8495 therm(THERM_PIN);

void setup() {
    Serial.begin(115200);
    while(Serial.available());
    Serial.println("Starting up...");

    pinMode(HEAT_PIN, OUTPUT);
    delay(1000);

}

unsigned long last_print = millis(), last_cycle = millis();
float goal = 60.0f;
int pwm_width = 5000, heat_duty = 0;
int on_time = pwm_width*(heat_duty/100.0f);
void loop() {
    //commands
    if(Serial.available()){
        String input = Serial.readString();
        switch(input.charAt(0)){
            case 't':
                goal = input.substring(1).toInt();
            break;
            case 'd':
                heat_duty = input.substring(1).toInt();
                on_time = pwm_width*(heat_duty/100.0f);
            break;
            case 'w':
                pwm_width = input.substring(1).toInt();
                on_time = pwm_width*(heat_duty/100.0f);
            break;
            default:
                Serial.println("unknown command");
            break;
        }
    }

    //data readout
    if(millis() - last_print > 250){
        Serial.println(therm.read());
        Serial.println(on_time);
        last_print = millis();
    }

    //heating
    if(therm.read() < goal){
        if(millis()-last_cycle > pwm_width){
            digitalWrite(HEAT_PIN, HIGH);
            last_cycle = millis();
        }else if(millis()-last_cycle > on_time){
            digitalWrite(HEAT_PIN, LOW);
        }
    }else{
        digitalWrite(HEAT_PIN, LOW);
    }

}