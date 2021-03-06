
/*
 *  AD8495 - Thermo couple breakout board
 *  Equations from : https://learn.adafruit.com/ad8495-thermocouple-amplifier/arduino
 *
 */

#ifndef _AD8495_H_
#define _AD8495_H_

#include <Arduino.h>
#include "temp_sense.h"

class AD8495 : public TempSensor{
  public:
    // Constructs a AD8495
    AD8495(int pin);

    // Reads temperature in degrees C
    float read();

  private:
    int pin_;
};

#endif /* _AD8495_H_*/
