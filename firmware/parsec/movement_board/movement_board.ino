
#include <Arduino.h>
#include <NUros.h>
#include <NU_GPIO.h>
#include <NU_DCMotor_DRV8701.h>
#include <NU_QuadratureEncoder.h>
#include <NU_NUDRV8701_Servo.h>

ros::NUNodeHandle nh;

NUGPIO led(nh, "led", PA9, OUTPUT);
NUDRV8701Servo carosel_mot(nh, "carosel", 2, PC14, PC15);//M2
NUDRV8701Driver brake1(nh, "brake1", 3);//M3
// NUDRV8701Driver brake2(nh, "brake2", 1);//M1

// NUGPIO carosel_lim(nh, "carosel_lim", PC14, INPUT_PULLUP);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    led.setup();
    carosel_mot.setup();
    brake1.setup();
}


void loop(){
    nh.spinOnce();
    led.update();
    carosel_mot.update();
    brake1.update();
}