#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <NU32Hardware.h>
#include <HX711.h>
#include <AccelStepper.h>

ros::NodeHandle_<NU32Hardware> nh;
std_msgs::Int64 pub_msg, stepper_msg;
ros::Publisher load_cell_pub("/load_cell", &pub_msg);
ros::Publisher stepper_pos_pub("/stepper_position", &stepper_msg);
ros::Subscriber load_cell_sub("/tare", &tare);
ros::Subscriber stepper_sub("/set_speed", &set_speed);
HX711 load_cell;
AccelStepper accel_stepper;

void setup() {
    nh.initNode();
    nh.advertise(load_cell_pub);
    nh.advertise(stepper_pos_pub);
    nh.subscribe(load_cell_sub);
    accel_stepper = AccelStepper();
    load_cell = HX711(6, 7);
}

void loop() {
    nh.spinOnce();
    stepper_msgs.data = accel_stepper.currentPosition();
    stepper_pos_pub.publish(&stepper_msg);
    pub_msg.data = load_cell.read();
    load_cell_pub.publish(&pub_msg);
    accel_stepper.runSpeed();
    delay(250);
}

void tare(std_msgs::UInt8 & tare_msg){
    load_cell.tare(tare_msg.data);
}

void set_speed(std_msgs::Float64 & speed_msg) {
    accel_stepper.setSpeed(speed_msg.data);
}