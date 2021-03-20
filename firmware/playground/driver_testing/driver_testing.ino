#define USE_USB

#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>

ros::NodeHandle nh;

NUGPIO led(nh, "led", 13);

void setup(){
    nh.initNode();
    led.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
}