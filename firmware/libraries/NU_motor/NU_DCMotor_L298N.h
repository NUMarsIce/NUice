#include <Arduino.h>
#include <NUros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

class DCMotorDriver: public NUDriver {
private:
    ros::Subscriber<std_msgs::Bool> dir_sub;
    ros::Subscriber<std_msgs::Int8> speed_sub;
    std_msgs::Bool direction_msg;
    int dir_pin_1, dir_pin_2, speed_pin;

public:
    DCMotorDriver(ros::NUNodeHandle& nh, const char* ns, int dir_pin_1, int dir_pin_2, int speed_pin): NUDriver(nh, ns) {
        dir_sub = ros::Subscriber<std_msgs::Bool>(appendNamespace("/motor_dir"), &set_direction);
        speed_sub = ros::Subscriber<std_msgs::Int8>(appendNamespace("/motor_speed"), &set_speed);
        this->dir_pin_1 = dir_pin_1;
        this->dir_pin_2 = dir_pin_2;
        this->speed_pin = speed_pin;
    }

    void setup() {
        pinMode(dir_pin_1, OUTPUT);
        pinMode(dir_pin_2, OUTPUT);
        nh.subscribe(dir_sub);
        nh.subscribe(speed_sub);
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

    void set_speed(const std_msgs::Int8 & speed_msg) {
        analogWrite(speed_pin, speed_msg.data);
    }
};