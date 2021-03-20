#include "NU_GPIO.h"


NUGPIO::NUGPIO(ros::NodeHandle& nh, const char* ns, uint8_t pin, uint8_t mode, uint8_t update_hz) 
              : NUDriver(nh, ns),
                state_pub_(appendNamespace("/current_state"), &state_pub_msg_),
                state_sub_(appendNamespace("/set_state"), &NUGPIO::setStateCb, this)
                {
    pin_ = pin;
    mode_ = mode;
    update_hz_ = update_hz;
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
    if(millis()-last_update > update_hz_){
        state_pub_.publish(&state_pub_msg_);
        last_update = millis();
    }
}

void NUGPIO::setStateCb(const std_msgs::Bool& state_msg){
    digitalWrite(pin_, state_msg.data);
}
