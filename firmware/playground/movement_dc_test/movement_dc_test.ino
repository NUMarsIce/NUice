#include <Arduino.h>

//PA6, PB10, PB13, PC7, PA8
//PA7, PB15, PB14, PC6, PC9
//PC4, PB0, PB1, PC8, PA10

#define CUR_PIN  PA6
#define PH_PIN  PB10
#define EN_PIN  PB13
#define NSLP_PIN PC7
#define NFLT_PIN PA8

void setup(){
    pinMode(PA9, OUTPUT);
    pinMode(CUR_PIN, INPUT);
    pinMode(PH_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    pinMode(NSLP_PIN, OUTPUT);
    pinMode(NFLT_PIN, INPUT);

    digitalWrite(PA9, HIGH);
    digitalWrite(NSLP_PIN, HIGH); //Enable
    digitalWrite(PH_PIN, HIGH); //Direction
    digitalWrite(EN_PIN, HIGH); //En/speed

}


void loop(){
    digitalWrite(PA9, HIGH);
    digitalWrite(PH_PIN, LOW); //Direction 1
    analogWrite(EN_PIN, 128);//speed
    delay(2000);

    digitalWrite(PA9, LOW);
    digitalWrite(PH_PIN, HIGH); //Direction 2
    analogWrite(EN_PIN, 128);//speed
    delay(2000);
}