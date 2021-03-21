
#include "NU_Heater.h"

NUHeater::NUHeater(ros::NodeHandle& nh, const char* ns, uint8_t heater_pin, TempSensor& temp_sensor, uint8_t update_hz)
              : NUDriver(nh, ns),
                temp_pub_(appendNamespace("/current_temp"), &temp_pub_msg_),
                state_pub_(appendNamespace("/current_state"), &state_pub_msg_),
                setpoint_sub_(appendNamespace("/set_setpoint"), &NUHeater::setpointCb, this){
    temp_sensor_ = temp_sensor;
    update_hz_ = update_hz;
    heater_pin_ = heater_pin;
}

void NUHeater::setup(){
    pinMode(heater_pin_, OUTPUT);
    digitalWrite(heater_pin_, LOW);
}

void NUHeater::update(){
    if(millis()-last_update_ > 1.0f/update_hz_){
        temp_pub_msg_.data = temp_sensor_.read();
        temp_pub_.publish(&temp_pub_msg_);

        //heating 
        if(temp_pub_msg_.data < setpoint_)
            digitalWrite(heater_pin_, HIGH);
        else
            digitalWrite(heater_pin_, LOW);

        last_update_ = millis();
    }
}

void NUHeater::setpointCb(const std_msgs::Float32& setpoint_msg){
    setpoint_ = setpoint_msg.data;
}
