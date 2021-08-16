#include <Arduino.h>
#include <NUros.h>

#include <NU_ACS712.h>

ros::NUNodeHandle nh;

NUACS712 power_5v(nh, "5v_supply", A1, false, ACS712_5A);
NUACS712 power_24v(nh, "24v_supply", A4, true, ACS712_20A);
NUACS712 total(nh, "mains", A3, true, ACS712_20A);
NUACS712 drill(nh, "drill", A2, true, ACS712_20A);

void setup(){
    nh.getHardware()->setBaud(115200);
    nh.setSpinTimeout(10); 
    nh.initNode();

    power_5v.setup();
    power_24v.setup();
    total.setup();
    drill.setup();
}

void loop(){
    nh.spinOnce();

    power_5v.update();
    power_24v.update();
    total.update();
    drill.update();
}


