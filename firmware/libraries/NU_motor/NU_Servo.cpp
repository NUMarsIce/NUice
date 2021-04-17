#include "NU_Servo.h"


NUServo::NUServo(ros::NUNodeHandle& nh, const char* ns, uint32_t pin, uint16_t min, uint16_t max, uint8_t update_hz) 
              : NUDriver(nh, ns),
                pos_pub_(appendNamespace("/current_position"), &pos_pub_msg_),
                stop_sub_(appendNamespace("/stop"), &NUServo::stopCb, this),
                pos_sub_(appendNamespace("/set_position"), &NUServo::setPositionCb, this),
                speed_sub_(appendNamespace("/set_speed"), &NUServo::setSpeedCb, this)
                {
    pin_ = pin;
    update_hz_ = update_hz;
    min_ = min;
    max_ = max;
}

void NUServo::setup(){
    servo_.attach(pin_, min_, max_);

    nh_.advertise(pos_pub_);
    nh_.subscribe(stop_sub_);
    nh_.subscribe(pos_sub_);
    nh_.subscribe(speed_sub_);
}

void NUServo::update(){
    //publish state
    if(millis()-last_update_ > 1000.0f/update_hz_){
        pos_pub_msg_.data = current_pos_;
        pos_pub_.publish(&pos_pub_msg_);

        last_update_ = millis();
    }

    //failsafe
    if(!nh_.connected() && goal_pos_ != current_pos_)
        goal_pos_ = current_pos_;

    //update servo pos at spped_ deg/s
    if(millis()-last_move_ > 1000.0f/speed_ && current_pos_ != goal_pos_){
        current_pos_ += (goal_pos_-current_pos_ > 0) ? 1 : -1;
        servo_.write(current_pos_);
    }

}

void NUServo::setPositionCb(const std_msgs::UInt16& pos_msg){
    goal_pos_ = pos_msg.data;
}   

void NUServo::setSpeedCb(const std_msgs::UInt8& speed_msg){
    speed_ = speed_msg.data;
}

void NUServo::stopCb(const std_msgs::Empty& stop_msg){
    goal_pos_ = current_pos_;
}
