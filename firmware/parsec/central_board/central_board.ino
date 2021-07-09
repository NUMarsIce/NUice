#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>
#include <NU_Stepper.h>
#include <NU_DCMotor_L298N.h>

ros::NUNodeHandle nh;

// NUGPIO led(nh, "led", PA9);

// Main steppers
NUStepper probe_stp(nh, "probe_stp", PC8, PC9, PA8); //J6: G, PA8,  PC9,  PC8,  PC7
NUStepper drill_stp(nh, "drill_stp", PA10, PC10, PC11); //J8: G, PC11, PC10, PA10, PA9
NUStepper cat_stp(nh, "cat_stp", PD2, PB4, PB5);     //J9: G, PB5,  PB4,  PD2,  PC12
NUGPIO probe_limit(nh, "probe_limit", PC7, INPUT_PULLUP, false, 10);
NUGPIO drill_limit(nh, "drill_limit", PA9, INPUT_PULLUP, false, 10);
NUGPIO cat_limit(nh, "cat_limit", PC12, INPUT_PULLUP, false, 10);

// Filtration relays TODO
// NUGPIO relay1(nh, "relay1", PA9);
// NUGPIO relay2(nh, "relay2", PA9);
// NUGPIO relay3(nh, "relay3", PA9);
// NUGPIO relay4(nh, "relay4", PA9);

// Filtration pumps
// L298NDriver pump1(nh, "pump1", PA1, PA2, PA3);
// L298NDriver pump2(nh, "pump2", PA1, PA2, PA3);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    // led.setup();
    probe_stp.setup();
    drill_stp.setup();
    cat_stp.setup();
    probe_limit.setup();
    drill_limit.setup();
    cat_limit.setup();
    // relay1.setup();
    // relay2.setup();
    // relay3.setup();
    // relay4.setup();
    // pump1.setup();
    // pump2.setup();
}


void loop(){
    nh.spinOnce();
    // led.update();
    probe_stp.setup();
    drill_stp.setup();
    cat_stp.setup();
    probe_limit.setup();
    drill_limit.setup();
    cat_limit.setup();
    // relay1.setup();
    // relay2.setup();
    // relay3.setup();
    // relay4.setup();
    // pump1.setup();
    // pump2.setup();
}