#include "NU_GPIO.h"


NUGPIO::NUGPIO(ros::NUNodeHandle& nh, const char* ns, uint32_t pin, uint32_t mode, bool analog, uint8_t update_hz) 
              : NUDriver(nh, ns),
                state_pub_(appendNamespace("/current_state"), &state_pub_msg_),
                state_sub_(appendNamespace("/set_state"), &NUGPIO::setStateCb, this),
                analog_pub_(appendNamespace("/current_analog"), &analog_pub_msg_),
                analog_sub_(appendNamespace("/set_analog"), &NUGPIO::setAnalogCb, this)
                {
    pin_ = pin;
    mode_ = mode;
    update_hz_ = update_hz;
    analog_ = analog;
}

void NUGPIO::setup(){
    pinMode(pin_, mode_);

    nh_.advertise(state_pub_);

    if(mode_ == OUTPUT && !analog_)
        nh_.subscribe(state_sub_);
    
    if(mode_ == OUTPUT && analog_)
        nh_.subscribe(analog_sub_);
    
    
}

void NUGPIO::update(){
    //publish state
    if(millis()-last_update_ > 1000.0f/update_hz_){
        state_pub_msg_.data = digitalRead(pin_);
        state_pub_.publish(&state_pub_msg_);

        if(mode_ != OUTPUT && analog_){
            analog_pub_msg_.data = analogRead(pin_);
            analog_pub_.publish(&analog_pub_msg_);
        }

        last_update_ = millis();
    }

    //failsafe
    if(!nh_.connected() && !digitalRead(pin_)){
        digitalWrite(pin_, LOW);
    }
}

void NUGPIO::setStateCb(const std_msgs::Bool& state_msg){
    digitalWrite(pin_, state_msg.data);
}

void NUGPIO::setAnalogCb(const std_msgs::UInt8& analog_msg){
    analogWrite(pin_, analog_msg.data);
}
