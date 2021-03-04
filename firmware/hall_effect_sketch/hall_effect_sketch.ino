#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <NU32Hardware.h>

ros::NodeHandle_<NU32Hardware> nh;
pin = #

std_msgs::Empty msg;
ros::Publisher pub("/hall_effect", &msg);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), publish, RISING);
  
}
void loop() {
  nh.spinOnce();
}

void publish() {
  pub.publish(&msg);
}
