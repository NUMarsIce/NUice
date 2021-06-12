#include "NU_UART.h"


NUUART::NUUART(int baud, uint8_t arr_size){
    baud_ = baud;
    size_ = arr_size;
    Serial.begin(baud);
    
    data_ = new int[size_];
}

NUUART::~NUUART(){
    delete[] data_;
}

NUUART::int parse(uint8_t idx){
    while(Serial.available()){
        String chunk = Serial.readStringUntil('\n');
        data[chunk.substring(0,1).toInt()] = chunk.substring(1).toInt();
    }

    return data[idx];
}

