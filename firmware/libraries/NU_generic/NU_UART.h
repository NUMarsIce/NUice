#include <Arduino.h>

class NUUART {
    public:
        NUUART(int baud=9600, uint8_t arr_size=3);
        ~NUUART();
        int parse(int idx);
        
    private:
        int baud_;
        int* data;      
}