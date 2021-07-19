#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <AccelStepper.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

class NUStepper : NUDriver{

public:
    NUStepper(ros::NUNodeHandle& nh, const char* ns, uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin_ = 0xFF, uint16_t max_speed = 400, uint16_t max_accel = 400);

    void setup();
    void update();

private:
    uint8_t step_pin_;
    uint8_t dir_pin_;
    uint8_t en_pin_;
    uint8_t max_speed_;
    uint8_t max_accel_;
    AccelStepper stepper_;
    
    uint8_t update_hz_ = 10;
    unsigned long last_update_ = millis();

    //Position Publisher
    std_msgs::Int32 pos_pub_msg_;
    ros::Publisher pos_pub_;

    //Enable Subscriber
    ros::Subscriber<std_msgs::Bool, NUStepper> en_sub_;
    void enableCb(const std_msgs::Bool& en_msg);

    //Absolute position Subscriber
    ros::Subscriber<std_msgs::Int32, NUStepper> abs_pos_sub_;
    void absolutePositionCb(const std_msgs::Int32& pos_msg);

    //Relative position Subscriber
    ros::Subscriber<std_msgs::Int32, NUStepper> rel_pos_sub_;
    void relativePositionCb(const std_msgs::Int32& rel_pos_msg);

    //Max speed Subscriber
    ros::Subscriber<std_msgs::UInt16, NUStepper> max_speed_sub_;
    void maxSpeedCb(const std_msgs::UInt16& max_speed_msg);

    //Acceleration Subscriber
    ros::Subscriber<std_msgs::UInt16, NUStepper> accel_sub_;
    void accelCb(const std_msgs::UInt16& accel_msg);

    //Stop Subscriber
    ros::Subscriber<std_msgs::Empty, NUStepper> stop_sub_;
    void stopCb(const std_msgs::Empty& stop_msg);
 

};