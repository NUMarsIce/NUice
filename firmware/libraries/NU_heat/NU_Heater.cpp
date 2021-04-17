
#include "NU_Heater.h"

NUHeater::NUHeater(ros::NUNodeHandle& nh, const char* ns, uint8_t heater_pin, TempSensor& temp_sensor, uint8_t update_hz)
              : NUDriver(nh, ns),
                temp_pub_(appendNamespace("/current_temp"), &temp_pub_msg_),
                state_pub_(appendNamespace("/current_state"), &state_pub_msg_),
                setpoint_sub_(appendNamespace("/set_setpoint"), &NUHeater::setpointCb, this),
                temp_sensor_(temp_sensor){
    update_hz_ = update_hz;
    heater_pin_ = heater_pin;
}

void NUHeater::setup(){
    nh_.advertise(temp_pub_);
    nh_.advertise(state_pub_);
    nh_.subscribe(setpoint_sub_);

    pinMode(heater_pin_, OUTPUT);
    digitalWrite(heater_pin_, LOW);
}

void NUHeater::update(){
    //update
    if(millis()-last_update_ > 1000.0f/update_hz_){
        temp_pub_msg_.data = temp_sensor_.read();
        temp_pub_.publish(&temp_pub_msg_);

        //heating 
        if(temp_pub_msg_.data < setpoint_)
            digitalWrite(heater_pin_, HIGH);
        else
            digitalWrite(heater_pin_, LOW);

        last_update_ = millis();
    }

    //failsafe
    if(!nh_.connected() && setpoint_ != 0)
        setpoint_ = 0;
    
}

void NUHeater::setpointCb(const std_msgs::Float32& setpoint_msg){
    setpoint_ = setpoint_msg.data;
}
