#define USE_USB

#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>
#include <NU_Heater.h>
#include <AD8495.h>

ros::NodeHandle nh;

NUGPIO led(nh, "led", 13, OUTPUT);
NULoadcell loadcell(nh, "loadcell", 2, 3, 6900.0f);
NUStepper stepper(nh, "stepper", 4, 5);

AD8495 heat_therm(6);
NUHeater heater(nh, "heater", 7, heat_therm);

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