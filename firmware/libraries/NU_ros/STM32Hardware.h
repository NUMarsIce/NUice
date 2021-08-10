/**
 * 
 * This defines how to send data from an NU32 over USBSerial 
 * 
 **/

#ifndef STM32_HARDWARE_H_
#define STM32_HARDWARE_H_

#define USB_MANUFACTURER_STRING "NUMarsIce"
#define USB_PRODUCT_STRING "NUBoard"

#include <Arduino.h>

class STM32Hardware
{
    public:
        STM32Hardware(int baud=57600){
            baud_=baud;
        }

        //set baud
        void setBaud(long baud){
        this->baud_= baud;
        }

        // any initialization code necessary to use the serial port
        void init(){
            Serial.begin(baud_);
        } 

        // read a byte from the serial port. -1 = failure
        int read(){
            return Serial.read();
        }

        // write data to the connection to ROS
        void write(uint8_t* data, int length){
        for(int i=0; i<length; i++)
            Serial.write(data[i]);
        }

        // returns milliseconds since start of program
        unsigned long time(){
            return millis();
        }

    private:
        int baud_ = 57600;

};

#endif
