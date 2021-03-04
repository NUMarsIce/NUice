#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <NU32Hardware.h>
#include <HX711.h>
#include <AccelStepper.h>

boolean tare(std_msgs::Empty &req, std_msgs::Empty &res){
    load_cell.tare();
    return true;
}

void set_speed(std_msgs::Float64 & speed_msg) {
    accel_stepper.setSpeed(speed_msg.data);
}

ros::NodeHandle_<NU32Hardware> nh;
std_msgs::Int64 pub_msg, stepper_msg;
ros::Publisher load_cell_pub("/load_cell", &pub_msg);
ros::Publisher stepper_pos_pub("/stepper_position", &stepper_msg);
ros::ServiceServer<std_msgs::Empty, std_msgs::Empty> server = nh.advertiseService("/tare", tare);
ros::Subscriber<std_msgs::Float64> stepper_sub("/set_speed", set_speed);
HX711 load_cell;
AccelStepper accel_stepper;

void setup() {
    nh.initNode();
    nh.advertise(load_cell_pub);
    nh.advertise(stepper_pos_pub);
    accel_stepper = AccelStepper(AccelStepper::FULL2WIRE, PA_2, PA_3);
    load_cell = HX711(PA_6, PA_7);
}

void loop() {
    nh.spinOnce();
    stepper_msg.data = accel_stepper.currentPosition();
    stepper_pos_pub.publish(&stepper_msg);
    pub_msg.data = load_cell.read();
    load_cell_pub.publish(&pub_msg);
    accel_stepper.runSpeed();
    delay(250);
}

