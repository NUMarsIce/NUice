#include "Switch.h"

Switch::Switch(int pin, int level = ACTIVE_HIGH){
    pin_ = pin;
    level_ = level;

    pinMode(pin_, INPUT);
}

bool Switch::read(){
    int state = digitalRead(pin_);
    // Using a ternary operator, return state depending on "on" level
    return (level_) ? state : !state;
}

void Switch::attachInterrupt(void(void) ISR){
    // Using a ternary operator, determine when to trigger the interrupt 
    // (we always want when the switch is activated)
    int edge = (level_) ? RISING : FALLING;
    attachInterrupt(digitalPinToInterrupt(pin_), ISR, edge);
}

void Switch::removeInterrupt(){
    detachInterrupt(digitalPinToInterrupt(pin_));
}
