#include "NU_Stepper.h"


NUStepper::NUStepper(ros::NUNodeHandle& nh, const char* ns, uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint16_t max_speed, uint16_t max_accel)
              : NUDriver(nh, ns),
                pos_pub_(appendNamespace("/current_position"), &pos_pub_msg_),
                abs_pos_sub_(appendNamespace("/set_abs_pos"), &NUStepper::absolutePositionCb, this),
                rel_pos_sub_(appendNamespace("/set_rel_pos"), &NUStepper::relativePositionCb, this),
                max_speed_sub_(appendNamespace("/set_max_speed"), &NUStepper::maxSpeedCb, this),
                accel_sub_(appendNamespace("/set_accel"), &NUStepper::accelCb, this),
                en_sub_(appendNamespace("/set_enabled"), &NUStepper::enableCb, this),
                stop_sub_(appendNamespace("/quick_stop"), &NUStepper::stopCb, this),
                stepper_(AccelStepper::DRIVER, step_pin, dir_pin){
    step_pin_ = step_pin;
    dir_pin_ = dir_pin;
    max_speed_ = max_speed;
    max_accel_ = max_accel_;
}

void NUStepper::setup(){
    nh_.advertise(pos_pub_);
    nh_.subscribe(abs_pos_sub_);
    nh_.subscribe(rel_pos_sub_);
    nh_.subscribe(max_speed_sub_);
    nh_.subscribe(accel_sub_);
    nh_.subscribe(en_sub_);
    nh_.subscribe(stop_sub_);

    stepper_.setMaxSpeed(max_speed_);
    stepper_.setAcceleration(max_accel_);
    stepper_.setEnablePin(en_pin_);
    stepper_.disableOutputs();
}

void NUStepper::update(){
    stepper_.run();

    if(millis()-last_update_ > 1000.0f/update_hz_){
        pos_pub_msg_.data = stepper_.currentPosition();
        pos_pub_.publish(&pos_pub_msg_);

        last_update_ = millis();
    }

    //failsafe
    if(!nh_.connected() && stepper_.isRunning())
        stepper_.stop();
    
}

void NUStepper::enableCb(const std_msgs::Bool& en_msg){
    if(en_msg.data)
        stepper_.enableOutputs();
    else
        stepper_.disableOutputs();
}

void NUStepper::absolutePositionCb(const std_msgs::Int32& pos_msg){
    stepper_.moveTo(pos_msg.data);
}

void NUStepper::relativePositionCb(const std_msgs::Int32& rel_pos_msg){
    stepper_.move(rel_pos_msg.data);
}

void NUStepper::maxSpeedCb(const std_msgs::UInt16& max_speed_msg){
    stepper_.setMaxSpeed(max_speed_msg.data);
}

void NUStepper::accelCb(const std_msgs::UInt16& accel_msg){
    stepper_.setAcceleration(accel_msg.data);
}

void NUStepper::stopCb(const std_msgs::Empty& stop_msg){
    stepper_.stop();
}   