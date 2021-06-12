#include <Arduino.h>

//PA_6, PB_10, PB_13, PC_7, PA_8
//PA_7, PB_15, PB_14, PC_6, PC_9
//PC_4, PB_0, PB_1, PC_8, PA_10

#define CUR_PIN  PA_6
#define IN1_PIN  PB_10
#define IN2_PIN  PB_13
#define NSLP_PIN PC_7
#define NFLT_PIN PA_8

void setup(){
    pinMode(PA_9, OUTPUT);
    pinMode(CUR_PIN, INPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(NSLP_PIN, OUTPUT);
    pinMode(NFLT_PIN, INPUT);

    digitalWrite(NSLP_PIN, HIGH); //Enable
    digitalWrite(IN1_PIN, LOW); 
    digitalWrite(IN2_PIN, HIGH); 

}


void loop(){
    // digitalWrite(PA_9, HIGH);
    // digitalWrite(IN1_PIN, LOW); //Direction 1
    // analogWrite(IN2_PIN, 128);
    // delay(1000);
    // digitalWrite(PA_9, LOW);
    // digitalWrite(IN2_PIN, LOW); //Direction 2
    // analogWrite(IN1_PIN, 128);
    // delay(1000);
}