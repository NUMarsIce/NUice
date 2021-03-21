#define USE_USB

#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>

ros::NodeHandle nh;

NUGPIO led(nh, "led", 13, OUTPUT);
NULoadcell loadcell(nh, "loadcell", 4, 5, 6900.0f);
NUStepper stepper(nh, "stepper", 2, 3);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    led.setup();
    loadcell.setup();
    stepper.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
    loadcell.update();
    stepper.update();
}