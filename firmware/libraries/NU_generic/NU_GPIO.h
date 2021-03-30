#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class NUGPIO : NUDriver{

public:
    NUGPIO(ros::NUNodeHandle& nh, const char* ns, uint8_t pin, uint8_t mode, bool analog = false, uint8_t update_hz = 2);

    void setup();
    void update();

private:
    uint32_t pin_;
    uint8_t mode_;
    uint8_t update_hz_;
    bool analog_;
    long last_update_ = millis();

    //Digital Publisher
    std_msgs::Bool state_pub_msg_;
    ros::Publisher state_pub_;
    //Digital Subscriber
    ros::Subscriber<std_msgs::Bool, NUGPIO> state_sub_;
    void setStateCb(const std_msgs::Bool& state_msg);

    //Analog Publisher
    std_msgs::UInt8 analog_pub_msg_;
    ros::Publisher analog_pub_;
    //Analog Subscriber
    ros::Subscriber<std_msgs::UInt8, NUGPIO> analog_sub_;
    void setAnalogCb(const std_msgs::UInt8& analog_msg);

};