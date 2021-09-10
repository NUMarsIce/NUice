#include <Arduino.h>
#include <NUros.h>

#include <NU_GPIO.h>

ros::NUNodeHandle nh;

NUGPIO led(nh, "led", PA9, OUTPUT);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    led.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
}