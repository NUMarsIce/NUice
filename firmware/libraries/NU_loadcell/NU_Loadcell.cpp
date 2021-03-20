
#include "NU_Loadcell.h"

NULoadcell::NULoadcell(ros::NodeHandle& nh, const char* ns, uint8_t dout_pin, uint8_t clk_pin, float scale)
              : NUDriver(nh, ns),
                load_pub_(appendNamespace("/load"), &load_pub_msg_),
                tare_sub_(appendNamespace("/tare"), &NULoadcell::tareCb, this){

    dout_pin_ = dout_pin;
    clk_pin_ = clk_pin;
    scale_ = scale;
}

void NULoadcell::setup(){
    loadcell_.begin(dout_pin_, clk_pin_);
    loadcell_.set_scale(scale_);
    loadcell_.tare();
}

void NULoadcell::update(){
    //publish load, needs to be called fast for some reason. Would be nice to untilize interrupts
    if(loadcell_.is_ready()){
        load_pub_msg_.data = loadcell_.get_units();
        load_pub_.publish(&load_pub_msg_);
    }
}

void NULoadcell::tareCb(const std_msgs::Empty& tare_msg){
    loadcell_.tare();
}

