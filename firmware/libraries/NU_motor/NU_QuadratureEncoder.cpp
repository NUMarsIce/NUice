#include "NU_QuadratureEncoder.h"


NUQuadratureEncoder::NUQuadratureEncoder(ros::NUNodeHandle& nh, const char* ns, uint32_t pin_a, uint32_t pin_b, uint32_t pin_z, uint8_t update_hz) 
              : NUDriver(nh, ns),
                pos_pub_(appendNamespace("/position"), &pos_pub_msg_),
                err_pub_(appendNamespace("/errors"), &err_pub_msg_),
                zero_sub_(appendNamespace("/zero"), &NUQuadratureEncoder::zeroCb, this)
                {
    pin_a_ = pin_a;
    pin_b_ = pin_b;
    update_hz_ = update_hz;
}

void NUQuadratureEncoder::setup(){
    encoder_ = new Encoders(pin_a_, pin_b_);//apparently we cant do this in the constructor rip

    nh_.advertise(err_pub_);
    nh_.advertise(pos_pub_);
    nh_.subscribe(zero_sub_);    
}

void NUQuadratureEncoder::update(){
    //publish position
    if(millis()-last_update_ > 1000.0f/update_hz_){
        pos_pub_msg_.data = encoder_->getEncoderCount();
        pos_pub_.publish(&pos_pub_msg_);

        err_pub_msg_.data = encoder_->getEncoderErrorCount();
        err_pub_.publish(&pos_pub_msg_);

        last_update_ = millis();
    }
}

void NUQuadratureEncoder::zeroCb(const std_msgs::Empty& state_msg){
    encoder_->setEncoderCount(0);
}

