#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>

class NUGPIO : NUDriver{

public:
    NUGPIO(ros::NodeHandle_<NU32Hardware> nh, String ns, uint8_t pin, uint8_t mode = OUTPUT);

    void setup();
    void update();

private:
    uint32_t pin_;
    uint8_t mode_;
    
    uint8_t pub_hz_ = 4;
    long last_pub = millis();

    std_msgs::Bool state_pub_msg_;
    ros::Publisher state_pub_;

    ros::Subscriber<std_msgs::Bool, NUGPIO> state_sub_;
    void setStateCb(const std_msgs::Bool& state_msg);

};