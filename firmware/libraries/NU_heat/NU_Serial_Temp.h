#pragma once
#include <NU_Serial.h>

class NUSerialTemp : public TempSensor{
    public:
        NUSerialTemp(NUSerial& ser, int idx) : ser_(ser), idx_(idx){}

        float read(){
            return ser_.parse(idx_);
        }

    private:
        NUSerial& ser_;
        int idx_;
};