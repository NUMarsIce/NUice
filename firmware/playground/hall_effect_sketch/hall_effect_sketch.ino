#include <Arduino.h>
#include <NUros.h>
#include <NU_HallEffect.h>

ros::NUNodeHandle nh;
HallEffectDriver he(nh, "hall_effect", 3);


void setup(){
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    he.setup();
}

void loop(){
    nh.spinOnce();
    he.update();
}