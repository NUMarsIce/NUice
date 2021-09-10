#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Float32.h>
#include "ACS712.h"

class NUACS712 : NUDriver{

public:
    NUACS712(ros::NUNodeHandle& nh, const char* ns, uint32_t pin, bool ac, uint16_t mV_per_amp = ACS712_5A, uint8_t update_hz = 10);

    void setup();
    void update();

private:
    bool ac_;
    uint32_t pin_;
    uint16_t mV_per_amp_;
    uint8_t update_hz_;
    long last_update_ = millis();

    ACS712 sensor_;

    //Analog Publisher
    std_msgs::Float32 current_pub_msg_;
    ros::Publisher current_pub_;
};