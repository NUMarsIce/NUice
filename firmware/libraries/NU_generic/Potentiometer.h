#pragma once

#include <Arduino.h>


class Potentiometer {
    public:
        /**
         * Constructor for a potentiometer, assuimg that it is setup with a balanced resitor devider (ex 10k pot and 10k resistor) 
         * Optionally takes the potentiometer resistance in ohms and that of the accompanying resistor (making up a resistor devider)
         */
        Potentiometer(int pin, float scaler = 1.0f, float offset = 0.0f);

        /**
         * Reads the potentiometer returning a value from 0.0 to 1.0
         */
        float read();

        /**
         * Sets the upper-bound and re-calculates the scaler
         */
        void setMax();
        /**
         * Sets the lower-bound and re-calculates the scaler
         */
        void setMin();

        /**
         * Gets the current scaler that determines the range of the pot
         */
        float getScaler();
        /**
         * Sets the current scaler
         */
        void setScaler(scaler);

        /**
         * Gets the current scaler that determines the range of the pot
         */
        float getOffset();
        /**
         * Sets the current scaler
         */
        void setOffset(scaler);
        
    private:
        int pin_;
        float scaler_;
        float offset_;
        float zero_ = 0;
}