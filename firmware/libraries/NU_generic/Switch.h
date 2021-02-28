#pragma once

#include <Arduino.h>

/**
 * Class to abstract simple switch functions such as state retrieval and interrupt attachment
 * 
 */

static const int ACTIVE_HIGH = 1;
static const int ACTIVE_LOW = 0;

class Switch{

    public:
        /**
         * Switch constructor. Takes a pin and optionally the switches "on" state/level.
         */
        Switch(int pin, int level = ACTIVE_HIGH);

        /**
         * Reads whether the switch is active.
         */
        bool read();
        
        /**
         * Attaches an interrupt to when the switch is activated.
         * ISR is the function that will be called when the switch is activated.
         */
        void attachInterrupt(void(void) ISR);

        /**
         * Removes the attached interrupt.
         */
        void removeInterrupt();

    private:
        int pin_;
        int level_;
};