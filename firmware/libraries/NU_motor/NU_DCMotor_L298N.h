#include <Arduino.h>
#include <NUros.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class L298NDriver : NUDriver {
private:
    ros::Subscriber<std_msgs::Bool, L298NDriver> dir_sub;
    ros::Subscriber<std_msgs::UInt8, L298NDriver> speed_sub;

    // Just for UI detection
    std_msgs::Bool fault_pub_msg_;
    ros::Publisher fault_pub;

    int dir_pin_1, dir_pin_2, speed_ = 0, direction_ = 0;

public:
    L298NDriver(ros::NUNodeHandle& nh, const char* ns, int dir_pin_1, int dir_pin_2) : 
                dir_sub(appendNamespace("/set_dir"), &L298NDriver::set_direction, this),
                speed_sub(appendNamespace("/set_speed"), &L298NDriver::set_speed, this),
                fault_pub(appendNamespace("/fault"), &fault_pub_msg_),
                NUDriver(nh, ns){

        this->dir_pin_1 = dir_pin_1;
        this->dir_pin_2 = dir_pin_2;
    }

    void setup() {
        pinMode(dir_pin_1, OUTPUT);
        pinMode(dir_pin_2, OUTPUT);

        digitalWrite(dir_pin_1, LOW);
        digitalWrite(dir_pin_2, LOW);

        nh_.subscribe(dir_sub);
        nh_.subscribe(speed_sub);

        nh_.advertise(fault_pub);
    }

    void update() {
        if(direction_) {
            analogWrite(dir_pin_1, speed_);
            digitalWrite(dir_pin_2, LOW);
        } else {
            digitalWrite(dir_pin_1, LOW);
            analogWrite(dir_pin_2, speed_);
        }
    }

    void set_direction(const std_msgs::Bool & dir_msg) {
        direction_ = dir_msg.data;
    }

    void set_speed(const std_msgs::UInt8 & speed_msg) {
        speed_ = speed_msg.data; 
    }
};