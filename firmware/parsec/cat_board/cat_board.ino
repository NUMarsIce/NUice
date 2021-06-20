#define USE_USB

#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>

ros::NodeHandle nh;

NUGPIO led(nh, "led", PA9);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    led.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
}