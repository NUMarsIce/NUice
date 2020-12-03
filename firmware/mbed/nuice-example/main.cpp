/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"
#include "USBSerial.h"


// Blinking rate in milliseconds
#define BLINKING_RATE_MS    500
USBSerial  serial;

int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(PA_9);
    DigitalOut led2(PA_10);
    AnalogIn   ain(PA_0);
    
    led2 = !led;
    while (true) {
        led = !led;
        led2 = !led2;
        
        serial.printf("%d\r\n", (int)(ain.read() * 10000));
        
        thread_sleep_for(BLINKING_RATE_MS);
    }
}
