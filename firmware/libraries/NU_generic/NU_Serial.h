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
        NUSerial() : serial(Serial){
            serial.begin(baud_);
            serial.setTimeout(1);//1ms timeout
        }

        NUSerial(uint8_t rx, uint8_t tx):serial(rx,tx){
            serial.begin(baud_);
            serial.setTimeout(1);//1ms timeout
        }

        void setBaud(int baud){
            baud_ = baud;
            Serial.flush();
            serial.begin(baud_);
        }

        int parse(int idx){
            while(serial.available()){
                int tmp = serial.parseInt();
                data_[tmp%10] = tmp/10;
            }
            return data_[idx];
        }
        
    private:
        HardwareSerial serial;

        int baud_ = 115200;
        int data_[10];      
};