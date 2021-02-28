
#include "Potentiometer.h"

Potentiometer(int pin, float scaler, float offset){
    pin_ = pin;
    scaler_ = scaler;
    offset_ = offset;

    pinMode(pin_, INPUT);
}

float read(){
    return analogRead(pin_)/1024f * scaler_;
}

void setMax(){
    max_ = analogRead(pin_);


}

void setMin(){

}


float getScaler(){
    return scaler_;
}

void setScaler(float scaler){
    scaler_ = scaler;
}