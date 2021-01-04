#include <Arduino.h>

void setup() {
    pinMode(PA_9, OUTPUT);
    pinMode(PA_10, OUTPUT);
    Serial.begin(9600);
}

bool led_state;
void loop() {
    led_state = !led_state;
    digitalWrite(PA_9, led_state);
    digitalWrite(PA_10, !led_state);
    Serial.println("Hello World");
    delay(1000);
    
}
