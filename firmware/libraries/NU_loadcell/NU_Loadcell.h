#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include "HX711.h"

class NULoadcell : NUDriver {
public:
    NULoadcell(ros::NUNodeHandle& nh, const char* ns, uint8_t dout_pin, uint8_t clk_pin, float scale);

    void setup();
    void update();

private:
    HX711 loadcell_;

    uint8_t dout_pin_;
    uint8_t clk_pin_;
    float scale_;
    uint8_t update_hz_;
    unsigned long last_update_ = millis();

    //Load publisher
    std_msgs::Float32 load_pub_msg_;
    ros::Publisher load_pub_;

    //Tare subscriber
    ros::Subscriber<std_msgs::Empty, NULoadcell> tare_sub_;
    void tareCb(const std_msgs::Empty& tare_msg);

};