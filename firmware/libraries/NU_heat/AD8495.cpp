#include "AD8495.h"

AD8495::AD8495(int pin) {
  pin_ = pin;
  pinMode(pin_, INPUT);
}

float AD8495::read() {
  float raw_adc = analogRead(pin_);
  float voltage = raw_adc * (5.0f / (pow(2, 10) - 1));
  float temp = (voltage - 1.25) / 0.005;
  return temp;
}
