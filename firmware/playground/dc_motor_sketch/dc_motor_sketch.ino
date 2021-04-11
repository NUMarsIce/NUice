#include <Arduino.h>
#include <NUros.h>
#include <NU_DCMotor.h>

ros::NUNodeHandle nh;
DCMotorDriver dc(nh, "dc_motor", #, #, #);

void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    dc.setup();
}

void loop() {
    nh.spinOnce();
    dc.update();
}
