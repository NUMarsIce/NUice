
#include <Arduino.h>
#include <NUros.h>

#include <NU_GPIO.h>
#include <NU_Loadcell.h>
#include <NU_Stepper.h>
#include <NU_Heater.h>
#include <AD8495.h>
#include <NU_Servo.h>
#include <NU_Serial.h>
#include <NU_Serial_Driver.h>

ros::NUNodeHandle nh;

// NUGPIO led(nh, "led", PA10, OUTPUT);
NULoadcell loadcell1(nh, "loadcell1", PB6, PB7, -6900.0f);//J10: 5v, PB6, PB7, PB8, PB9
NULoadcell loadcell2(nh, "loadcell2", PB8, PB9, -6900.0f);// Uncalibrated, but 150 is a good upper maximum
NUStepper rot_stp(nh, "rot_stp", PC8, PC9, PA8); //J6: G, PA8,  PC9,  PC8,  PC7
NUStepper pitch_stp(nh, "pitch_stp", PD2, PB4, PB5); //J9: G, PB5,  PB4,  PD2,  PC12

NUSerial ser(PA10, PA9); //USART1, J8: 5v, PA9, PA10
NUSerialDriver therm1(nh, "probe_therm1", ser, 0);
NUSerialDriver therm2(nh, "probe_therm2", ser, 1);
NUSerialDriver pitch_pot(nh, "pitch_pot", ser, 2, true);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.setSpinTimeout(10); 
    nh.initNode();

    // led.setup();
    loadcell1.setup();
    loadcell2.setup();
    rot_stp.setup();
    pitch_stp.setup();
    pitch_pot.setup();
    therm1.setup();
    therm2.setup();
    ser.begin();
}

void loop(){
    nh.spinOnce();

    // led.setup();
    loadcell1.update();
    loadcell2.update();
    rot_stp.update();
    pitch_stp.update();
    pitch_pot.update();
    therm1.update();
    therm2.update();
    ser.update();

}