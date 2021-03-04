#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <NU32Hardware.h>
#include <HX711.h>
#include <AccelStepper.h>

void tare(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
void set_speed(const std_msgs::Float64 & speed_msg);

ros::NodeHandle_<NU32Hardware> nh;
std_msgs::Int64 pub_msg;
std_msgs::Float64 pub_msg;
ros::Publisher load_cell_pub("/load_cell", &pub_msg);
ros::Publisher stepper_pos_pub("/stepper_position", &stepper_msg);
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> stepper_pos_srv("/tare", &tare);
ros::Subscriber<std_msgs::Float64> stepper_sub("/set_speed", &set_speed);

HX711 load_cell;
AccelStepper accel_stepper;

void setup() {
    nh.initNode();
    nh.advertise(load_cell_pub);
    nh.advertise(stepper_pos_pub);
    nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(stepper_pos_srv);
    accel_stepper = AccelStepper(AccelStepper::FULL2WIRE, PA_2, PA_3);
    load_cell = HX711(PA_6, PA_7);
    load_cell.set_scale(-6900);
}

void loop() {
    nh.spinOnce();
    stepper_msg.data = accel_stepper.currentPosition();
    stepper_pos_pub.publish(&stepper_msg);
    pub_msg.data = load_cell.getUnits();
    load_cell_pub.publish(&pub_msg);
    accel_stepper.runSpeed();
    delay(250);
}

void tare(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res){
    load_cell.tare();
}

void set_speed(const std_msgs::Float64 & speed_msg) {
    accel_stepper.setSpeed(speed_msg.data);
}