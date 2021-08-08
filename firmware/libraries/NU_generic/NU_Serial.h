/**
 * 
 * Simple serial communication protocol that transfers integers, where the last digit is an index (aka max 10 integers).
 * See melt_nano.ino for example
 * 
 */
#pragma once
#include <Arduino.h>
#include "CRC.h"

#define NUSER_MAX_LEN 10
#define NUSER_TIMEOUT 10
class NUSerial {
    public:
#if defined(STM32F4xx)
        NUSerial(uint32_t rx, uint32_t tx){
            serial = new HardwareSerial(rx,tx);
            serial->setTimeout(NUSER_TIMEOUT);
        }
#else
        NUSerial(){
            serial = &Serial;
            serial->setTimeout(NUSER_TIMEOUT);
        }
#endif 

        void begin(){
            serial->begin(baud_);
        }

        void setBaud(int baud){
            baud_ = baud;
            serial->flush();
            serial->begin(baud_);
        }

        void update(){
            while(serial->available()){
                if(serial->read() == 2){ //start of read, pop out of buffer (removes end and bad bytes)
                    uint8_t read_buff[6];//indx, int, crc
                    serial->readBytes(read_buff, 6);

                    //check crc
                    if(read_buff[5] != crc8(read_buff, 5)){
                        errors_++;
                        break;
                    }

                    //extract payload
                    uint32_t data = ((uint32_t)read_buff[4]<<24) | ((uint32_t)read_buff[3]<<16) | ((uint32_t)read_buff[2]<<8) | ((uint32_t)read_buff[1]);
                    
                    //save
                    data_[255-read_buff[0]] = data;

                }
            }
        }

        int read(uint8_t idx){
            return data_[idx];
        }

        int send(uint8_t idx, int32_t data){
            uint8_t* data_bytes = (uint8_t*)&data;
            uint8_t payload_buff_[5]; //idx,len,int

            //fill data buffer
            payload_buff_[0]= 255-idx;//to keep value away from the start byte
            payload_buff_[1] = data_bytes[0];
            payload_buff_[2] = data_bytes[1];
            payload_buff_[3] = data_bytes[2];
            payload_buff_[4] = data_bytes[3];

            //calculate crc
            uint8_t crc = crc8(payload_buff_, 5);

            //send data
            serial->write(2);                //start byte STX
            serial->write(payload_buff_, 5);//payload
            serial->write(crc);              //crc
            serial->write('\n');             //stop byte
        }
        
        uint32_t get_errors(){
            return errors_;
        }

    private:

        HardwareSerial* serial;

        uint32_t baud_ = 115200;
        int32_t data_[NUSER_MAX_LEN]; 
        uint32_t errors_ = 0;     
};