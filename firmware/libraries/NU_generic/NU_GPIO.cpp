#include "NU_GPIO.h"


NUGPIO::NUGPIO(ros::NodeHandle_<NU32Hardware> nh, String ns, uint8_t pin, uint8_t mode) 
              : state_pub_(String(ns+"/current_state").c_str(), &state_pub_msg_),
                state_sub_(String(ns+"/set_state").c_str(), &NUGPIO::setStateCb, this),
                NUDriver(nh, ns){
    pin_ = pin;
    mode_ = mode;
}


void NUGPIO::setup(){
    pinMode(pin_, mode_);

    nh_.advertise(state_pub_);

    if(mode_ == OUTPUT){
        nh_.subscribe(state_sub_);
    }
}


void NUGPIO::update(){
    //publish state
    if(millis()-last_pub > pub_hz_){
        state_pub_.publish(&state_pub_msg_);
        last_pub = millis();
    }
}

void NUGPIO::setStateCb(const std_msgs::Bool& state_msg){
    digitalWrite(pin_, state_msg.data);
}