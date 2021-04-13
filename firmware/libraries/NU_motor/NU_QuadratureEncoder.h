#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include "QuadratureEncoder.h"
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>

class NUQuadratureEncoder : NUDriver{

public:
    NUQuadratureEncoder(ros::NUNodeHandle& nh, const char* ns, uint32_t pin_a, uint32_t pin_b, uint32_t pin_z = 0xFF, uint8_t update_hz = 2);

    void setup();
    void update();

private:
    uint32_t pin_a_;
    uint32_t pin_b_;
    uint32_t pin_z_;
    uint8_t update_hz_;
    long last_update_ = millis();

    Encoders encoder_;

    //Position Publisher
    std_msgs::Int32 pos_pub_msg_;
    ros::Publisher pos_pub_;

    //Error Publisher
    std_msgs::UInt32 err_pub_msg_;
    ros::Publisher err_pub_;

    //Digital Subscriber
    ros::Subscriber<std_msgs::Empty, NUQuadratureEncoder> zero_sub_;
    void zeroCb(const std_msgs::Empty& zero_msg);
};