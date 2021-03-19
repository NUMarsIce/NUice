#pragma once

#include <Arduino.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

class NUGPIO : NUDriver{

public:
    NUGPIO(ros::NodeHandle_<NU32Hardware> nh, String ns, uint8_t pin, uint8_t mode = OUTPUT);

    void setup();
    void update();

private:
    uint32_t pin_;
    uint8_t mode_;
    int update_hz_;
    long last_update = millis();

    std_msgs::Bool state_pub_msg_;
    ros::Publisher state_pub_;

    ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse, NUGPIO> state_srv_;
    void setStateSrvCb(const std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&);

    ros::Subscriber<std_msgs::Bool, NUGPIO> state_sub_;
    void setStateCb(const std_msgs::Bool& state_msg);

};