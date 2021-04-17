#include <Arduino.h>
#include <NUros.h>
#include <NU_DCMotor_L298N.h>

ros::NUNodeHandle nh;
DCMotorDriver dc(nh, "dc_motor", PA_5, PA_6, PA_7);

void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    dc.setup();
}

void loop() {
    nh.spinOnce();
    dc.update();
}
