#pragma once
#include <NU_UART.h>

class NUUARTTemp : TempSensor{
    public:
        NUUARTTemp(NUUART& uart, int idx){
            uart_ = uart;
            idx_ = idx;
        }

        float read(){
            return uart.parse(idx_);
        }

    private:
        NUUART& uart_;
        int idx_;
};