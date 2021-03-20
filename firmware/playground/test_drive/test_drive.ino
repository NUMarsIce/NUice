#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

#include <HX711.h>
#include <AccelStepper.h>

const int LOADCELL_DOUT_PIN = PA_6;
const int LOADCELL_SCK_PIN = PA_7;

//For testing with the arduino mega
ros::NodeHandle nh;

void tare(const std_msgs::Empty & tare_msg);
void set_speed(const std_msgs::Float64 & speed_msg);
void move(const std_msgs::Float64 & move_msg);

std_msgs::Float64 stepper_msg;
std_msgs::Float64 pub_msg;
ros::Publisher load_cell_pub("/load_cell", &pub_msg);

ros::Publisher stepper_pos_pub("/stepper_position", &stepper_msg);
ros::Subscriber<std_msgs::Empty> tare_sub("/tare", &tare);

ros::Subscriber<std_msgs::Float64> stepper_speed_sub("/set_speed", &set_speed);
ros::Subscriber<std_msgs::Float64> move_sub("/set_move", &move); 

HX711 load_cell;
AccelStepper accel_stepper;

void setup() {
    nh.initNode();
    nh.advertise(load_cell_pub);
    nh.advertise(stepper_pos_pub);
    nh.subscribe(tare_sub);
    nh.subscribe(stepper_speed_sub);
    nh.subscribe(move_sub);

    accel_stepper = AccelStepper(AccelStepper::DRIVER, PA_2, PA_3);
    accel_stepper.setMaxSpeed(500);
    accel_stepper.setAcceleration(200);

    load_cell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    load_cell.set_scale(2280.f);
    load_cell.tare();

}

long last_pub = millis();
void loop() {
    nh.spinOnce();
    accel_stepper.run();

    if(millis()-last_pub > 100){
        stepper_msg.data = accel_stepper.currentPosition()/200.0f*3.4395833;
        stepper_pos_pub.publish(&stepper_msg);
        last_pub = millis();
    }

    //Needs to run as fast as possible apparently. Would be nice to utilise interrupts instead
    if(load_cell.is_ready()){
        pub_msg.data = load_cell.get_units();
        load_cell_pub.publish(&pub_msg);
    }

}

void tare(const std_msgs::Empty & tare_msg){
    load_cell.tare();
}

void set_speed(const std_msgs::Float64 & speed_msg) {
    accel_stepper.setMaxSpeed(speed_msg.data);
}

void move(const std_msgs::Float64 & move_msg) {
    accel_stepper.moveTo(move_msg.data*200/3.4395833);
}