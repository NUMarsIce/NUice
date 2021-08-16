#include "NU_ACS712.h"

NUACS712::NUACS712(ros::NUNodeHandle& nh, const char* ns, uint32_t pin, bool ac, uint16_t mV_per_amp, uint8_t update_hz) 
              : NUDriver(nh, ns),
                current_pub_(appendNamespace("/current"), &current_pub_msg_),
                sensor_(pin, mV_per_amp)
                {
    pin_ = pin;
    mV_per_amp_ = mV_per_amp;
    update_hz_ = update_hz;
    ac_ = ac;
}

void NUACS712::setup(){
    pinMode(pin_, INPUT);

    nh_.advertise(current_pub_);    
}

void NUACS712::update(){
    //publish current
    if(millis()-last_update_ > 1000.0f/update_hz_){
        current_pub_msg_.data = ac_ ? sensor_.readAC() : sensor_.read();
        current_pub_.publish(&current_pub_msg_);

        last_update_ = millis();
    }
}
