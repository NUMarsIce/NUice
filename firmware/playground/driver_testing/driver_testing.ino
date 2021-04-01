
#include <Arduino.h>
#include <NUros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>
#include <NU_Heater.h>
#include <AD8495.h>

ros::NUNodeHandle nh;

NUGPIO led(nh, "led", PA_9, OUTPUT);
NULoadcell loadcell(nh, "loadcell", PA_6, PA_7, 6900.0f);
// NUStepper stepper(nh, "stepper", PA_2, PA_3);

// AD8495 heat_therm(PB_0);
// NUHeater heater(nh, "heater", PB_1, heat_therm);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    led.setup();
    loadcell.setup();
    // stepper.setup();
    // heater.setup();
}

void loop(){
    nh.spinOnce();
    led.update();
    // loadcell.update();
    // stepper.update();
    // heater.update();
}