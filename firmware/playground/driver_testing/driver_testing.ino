#include <Arduino.h>
#include <ros.h>
#include <NU32Hardware.h>
#include <NU_GPIO.h>

ros::NodeHandle_<NU32Hardware> nh;

NUGPIO led(nh, "ledA9", PA_9);

void setup(){
    nh.initNode();
    led.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
}