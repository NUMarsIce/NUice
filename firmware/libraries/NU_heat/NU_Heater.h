#pragma once

#include <NU_Driver.h>
#include "temp_sense.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

class NUHeater : NUDriver{
public:
    NUHeater(ros::NUNodeHandle& nh, const char* ns, uint8_t heater_pin, TempSensor& temp_sensor, uint8_t update_hz = 5);

    void setup();
    void update();

private:
    TempSensor& temp_sensor_;
    uint8_t heater_pin_;
    uint8_t update_hz_;
    float setpoint_ = 0.0;
    unsigned long last_update_ = millis();
    unsigned long last_cycle_ = millis();

    //Temp Publisher
    std_msgs::Float32 temp_pub_msg_;
    ros::Publisher temp_pub_;
    //State Publisher
    std_msgs::Bool state_pub_msg_;
    ros::Publisher state_pub_;

    //Setpoint Subscriber
    ros::Subscriber<std_msgs::Float32, NUHeater> setpoint_sub_;
    void setpointCb(const std_msgs::Float32& setpoint_msg);

};