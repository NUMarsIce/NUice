#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

class NUGPIO : NUDriver{

public:
    NUGPIO(ros::NodeHandle& nh, const char* ns, uint8_t pin, uint8_t mode = OUTPUT, uint8_t update_hz = 2);

    void setup();
    void update();

private:
    uint32_t pin_;
    uint8_t mode_;
    uint8_t update_hz_;
    long last_update = millis();

    std_msgs::Bool state_pub_msg_;
    ros::Publisher state_pub_;

    ros::Subscriber<std_msgs::Bool, NUGPIO> state_sub_;
    void setStateCb(const std_msgs::Bool& state_msg);

};