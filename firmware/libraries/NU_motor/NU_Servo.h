#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Empty.h>
#include <Servo.h>

class NUServo : NUDriver{

public:
    NUServo(ros::NUNodeHandle& nh, const char* ns, uint32_t pin, uint16_t min = 544, uint16_t max = 2400, uint8_t update_hz = 2);

    void setup();
    void update();

private:
    uint32_t pin_;
    uint16_t min_, max_;
    uint8_t update_hz_;
    uint8_t current_pos_ = 0, goal_pos_ = 0, speed_ = 0;
    long last_update_ = millis();
    long last_move_ = millis();

    Servo servo_;

    //Position Publisher
    std_msgs::UInt16 pos_pub_msg_;
    ros::Publisher pos_pub_;

    //Position Subscriber
    ros::Subscriber<std_msgs::UInt16, NUServo> pos_sub_;
    void setPositionCb(const std_msgs::UInt16& pos_msg);
    //Speed Subscriber
    ros::Subscriber<std_msgs::UInt8, NUServo> speed_sub_;
    void setSpeedCb(const std_msgs::UInt8& speed_msg);
    //Stop Subscriber
    ros::Subscriber<std_msgs::Empty, NUServo> stop_sub_;
    void stopCb(const std_msgs::Empty& stop_msg);
};