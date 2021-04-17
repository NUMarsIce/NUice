
#include <Arduino.h>
#include <NUros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>

ros::NUNodeHandle nh;

NUGPIO drill(nh, "drill", PA_5, OUTPUT);
NULoadcell loadcell(nh, "loadcell", PA_6, PA_7, 2915.0f);
NUStepper stepper(nh, "stepper", PA_10, PA_9);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.setSpinTimeout(100); //Stop spinOnce from taking up too much time. Failsafe but shouldn't happen
    nh.initNode();

    drill.setup();
    loadcell.setup();
    stepper.setup();
}

void loop(){
    nh.spinOnce();
    drill.update();
    loadcell.update();
    stepper.update();
}