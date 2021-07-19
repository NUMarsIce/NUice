#include <Arduino.h>
#include <ros.h>
#include <NU_GPIO.h>
#include <NU_Stepper.h>
#include <NU_DCMotor_L298N.h>

ros::NUNodeHandle nh;

// NUGPIO led(nh, "led", PA10);

// Main steppers
NUStepper probe_stp(nh, "probe_stp", PC8, PC9, PA8); //J6: G, PA8,  PC9,  PC8,  PC7
NUStepper drill_stp(nh, "drill_stp", PA10, PC10, PC11); //J8: G, PC11, PC10, PA10, PA9
NUStepper cat_stp(nh, "cat_stp", PD2, PB4, PB5);     //J9: G, PB5,  PB4,  PD2,  PC12
NUGPIO probe_limit(nh, "probe_limit", PC7, INPUT_PULLUP, false, 10);
NUGPIO drill_limit(nh, "drill_limit", PA9, INPUT_PULLUP, false, 10);
NUGPIO cat_limit(nh, "cat_limit", PC12, INPUT_PULLUP, false, 10);

// Main Relays J5: PC6, PB15, PB14, PB13
NUGPIO heater1_relay(nh, "heater1_relay", PB15, OUTPUT);
NUGPIO heater2_relay(nh, "heater2_relay", PC6, OUTPUT);
NUGPIO drill_relay(nh, "drill_relay", PB13, OUTPUT);
NUGPIO power_relay(nh, "power_relay", PB14, OUTPUT);

// Filtration relays TODO
// NUGPIO relay1(nh, "relay1", PA9, OUTPUT);
// NUGPIO relay2(nh, "relay2", PA9, OUTPUT);
// NUGPIO relay3(nh, "relay3", PA9, OUTPUT);
// NUGPIO relay4(nh, "relay4", PA9, OUTPUT);

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
    heater1_relay.setup();
    heater2_relay.setup();
    drill_relay.setup();
    power_relay.setup();
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
    probe_stp.update();
    drill_stp.update();
    cat_stp.update();
    probe_limit.update();
    drill_limit.update();
    cat_limit.update();
    heater1_relay.update();
    heater2_relay.update();
    drill_relay.update();
    power_relay.update();
    // relay1.update();
    // relay2.update();
    // relay3.update();
    // relay4.update();
    // pump1.update();
    // pump2.update();
}