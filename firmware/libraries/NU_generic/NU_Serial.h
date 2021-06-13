/**
 * 
 * Simple serial communication protocol that transfers integers, where the last digit is an index (aka max 10 integers).
 * See melt_nano.ino for example
 * 
 */
#pragma once
#include <Arduino.h>

class NUSerial {
    public:
        NUSerial(){
            baud_ = 9600;
            Serial.begin(baud_);
        }

        NUSerial(int baud){
            baud_ = baud;
            Serial.begin(baud_);
        }

        int parse(int idx){
            while(Serial.available()){
                int tmp = Serial.parseInt();
                data_[tmp%10] = tmp/10;
            }
            return data_[idx];
        }
        
    private:
        int baud_;
        int data_[10];      
};