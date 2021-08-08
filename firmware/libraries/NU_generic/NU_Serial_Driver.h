
#pragma once
#include "NU_Driver.h"
#include "NU_Serial.h"
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>

class NUSerialDriver : public NUDriver {
    
public:
    NUSerialDriver(ros::NUNodeHandle& nh, const char* name, NUSerial& ser, uint8_t idx, bool debug = false, uint8_t update_hz = 4) : 
        NUDriver(nh, name), 
        ser_(ser), idx_(idx), debug_(debug), update_hz_(update_hz), 
        pub_(appendNamespace("/value"), &pub_msg_), 
        err_pub_(appendNamespace("/err"), &err_pub_msg_) 
    {}

    virtual void setup() {
       nh_.advertise(pub_); 
       if(debug_) nh_.advertise(err_pub_); 
    }

    virtual void update() {

        if(millis()-last_update_ > 1000.0f/update_hz_){
            pub_msg_.data = ser_.read(idx_);
            pub_.publish(&pub_msg_);
            
            if(debug_){
                err_pub_msg_.data = ser_.get_errors();
                err_pub_.publish(&err_pub_msg_);
            } 

            last_update_ = millis();
        }
    }

private:
    const char* name_;
    NUSerial& ser_;
    uint8_t idx_;
    uint8_t update_hz_;
    unsigned long last_update_ = millis();
    bool debug_;

    //Publisher
    std_msgs::UInt32 pub_msg_;
    ros::Publisher pub_;

    std_msgs::UInt32 err_pub_msg_;
    ros::Publisher err_pub_;

};

