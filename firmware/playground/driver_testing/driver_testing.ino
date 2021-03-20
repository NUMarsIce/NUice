#define USE_USB

#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>

ros::NodeHandle nh;

NUGPIO led(nh, "led", 13, OUTPUT);
NULoadcell loadcell(nh, "loadcell", 4, 5, 6900.0f);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    led.setup();
    loadcell.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
    loadcell.update();
}