
#include <Arduino.h>
#include <NUros.h>
#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>
#include <NU_Heater.h>
#include <AD8495.h>
#include <NU_Servo.h>
#include <NU_Serial.h>
#include <NU_Serial_Temp.h>

ros::NUNodeHandle nh;

NUGPIO led(nh, "led", PA_10, OUTPUT);
NULoadcell loadcell1(nh, "loadcell1", PA_6, PA_7, 6900.0f);
NULoadcell loadcell2(nh, "loadcell2", PA_6, PA_7, 6900.0f);
NUStepper rot_stp(nh, "rot_stp", PA_2, PA_3);
NUStepper pitch_stp(nh, "pitch_stp", PA_2, PA_3);

NUSerial ser;
NUSerialTemp therm1(ser, 0);
NUSerialTemp therm2(ser, 1);
NUSerialTemp pot(ser, 3);

NUHeater heater1(nh, "heater1", PB_1, therm1);
NUHeater heater2(nh, "heater2", PB_1, therm2);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.setSpinTimeout(10); 
    nh.initNode();

    led.setup();
    loadcell1.setup();
    loadcell2.setup();
    rot_stp.setup();
    pitch_stp.setup();
    heater1.setup();
    heater2.setup();
}

void loop(){
    nh.spinOnce();

    led.setup();
    loadcell1.update();
    loadcell2.update();
    rot_stp.update();
    pitch_stp.update();
    heater1.update();
    heater2.update();
}