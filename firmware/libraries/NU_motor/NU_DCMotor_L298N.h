#include <Arduino.h>
#include <NUros.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class L298NDriver : NUDriver {
private:
    ros::Subscriber<std_msgs::Bool, L298NDriver> dir_sub;
    ros::Subscriber<std_msgs::UInt8, L298NDriver> speed_sub;
    int dir_pin_1, dir_pin_2, speed_pin;

public:
    L298NDriver(ros::NUNodeHandle& nh, const char* ns, int dir_pin_1, int dir_pin_2, int speed_pin) : 
                dir_sub(appendNamespace("/motor_dir"), &L298NDriver::set_direction, this),
                speed_sub(appendNamespace("/motor_speed"), &L298NDriver::set_speed, this),
                NUDriver(nh, ns){

        this->dir_pin_1 = dir_pin_1;
        this->dir_pin_2 = dir_pin_2;
        this->speed_pin = speed_pin;
    }

    void setup() {
        pinMode(dir_pin_1, OUTPUT);
        pinMode(dir_pin_2, OUTPUT);

        digitalWrite(dir_pin_1, HIGH);
        digitalWrite(dir_pin_2, LOW);

        nh_.subscribe(dir_sub);
        nh_.subscribe(speed_sub);
    }

    void update() {}

    void set_direction(const std_msgs::Bool & dir_msg) {
        if(dir_msg.data) {
            digitalWrite(dir_pin_1, HIGH);
            digitalWrite(dir_pin_2, LOW);
        } else {
            digitalWrite(dir_pin_1, LOW);
            digitalWrite(dir_pin_2, HIGH);
        }
    }

    void set_speed(const std_msgs::UInt8 & speed_msg) {
        analogWrite(speed_pin, speed_msg.data);
    }
};