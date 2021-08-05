
#pragma once
#include "NU_Driver.h"
#include "NU_Serial.h"
#include <std_msgs/Int32.h>

class NUSerialDriver : public NUDriver {
    
public:
    NUSerialDriver(ros::NUNodeHandle& nh, const char* name, NUSerial& ser, uint8_t idx, uint8_t update_hz = 4) : 
        NUDriver(nh, ""), 
        ser_(ser), name_(name), update_hz_(update_hz), idx_(idx),
        pub_(name_, &pub_msg_) 
    {}

    virtual void setup() {
       nh_.advertise(pub_); 
    }

    virtual void update() {
        pub_msg_.data = ser_.parse(idx_);

        if(millis()-last_update_ > 1000.0f/update_hz_){
            pub_.publish(&pub_msg_);

            last_update_ = millis();
        }
    }

private:
    const char* name_;
    NUSerial& ser_;
    uint8_t idx_;
    uint8_t update_hz_;
    unsigned long last_update_ = millis();

    //Digital Publisher
    std_msgs::Int32 pub_msg_;
    ros::Publisher pub_;

};

