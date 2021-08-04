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
        NUSerial() : serial(Serial){
            serial.begin(baud_);
            serial.setTimeout(NUSER_TIMEOUT);
        }

        NUSerial(uint8_t rx, uint8_t tx):serial(rx,tx){
            serial.begin(baud_);
            serial.setTimeout(NUSER_TIMEOUT);
        }

        void setBaud(int baud){
            baud_ = baud;
            serial.flush();
            serial.begin(baud_);
        }

        void update(){
            while(serial.available()){
                if(serial.read() == 2){ //start of read
                    uint8_t read_buff[3+10+1]; 
                    uint8_t idx = read_buff[0] = serial.read();
                    uint8_t len = read_buff[1] = serial.read();
                    uint8_t sign = read_buff[2] = serial.read();
                    serial.readBytes(read_buff+3, len+1);

                    //check crc
                    if(read_buff[len]!=crc8(read_buff,len+3)){
                        errors_++;
                        break;
                    }

                    //extract payload
                    int32_t data = 0;
                    for(int i = 0; i < len+1; i++)
                        data = data*10+read_buff[i+3];
                    if(sign == '-')
                        data *= -1;
                    
                    //save
                    data_[idx] = data;

                }else{//bad data
                    serial.read();  //pop off bad data
                }
            }
        }

        int read(uint8_t idx){
            return data_[idx];
        }

        int send(uint8_t idx, int32_t data){
            uint8_t payload_buff_[1+1+11]; //idx,len,int
            int tmp = data;
            uint8_t len;
            while ( tmp /= 10 )
                len++;

            //fill data buffer
            data_buff_[0]=idx;
            data_buff_[1]=len;
            data_buff_[2]=(data<0)? '-':'+';

            for(uint8_t i = 1; i <= len; i++){
                data_buff_[len+3-i] = (data%10);
                data /= 10;
            }

            //calculate crc
            uint8_t crc = crc8(payload_buff_, len+3);

            //send data
            serial.write(2);                //start byte STX
            serial.write(data_buff_, len+3);//payload
            serial.write(crc);              //crc
            serial.write('\n');             //stop byte
        }
        
    private:
        HardwareSerial serial;

        uint32_t baud_ = 9600;
        int32_t data_[NUSER_MAX_LEN]; 
        uint32_t errors_ = 0;     
};