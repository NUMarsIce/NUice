#ifndef _ACS712_H_
#define _ACS712_H_

#include <Arduino.h>

/**
 * Driver for the ACS712 Current sensor. This sensor comes in multiple current ratings
 * Author: Ian Burwell 2/3/21
 */

//mV per Amp for different varients
static const int ACS712_5A = 185;
static const int ACS712_20A = 100;
static const int ACS712_30A = 66;

class ACS712 {
  public:

    /**
     * 
     */
    ACS712(int pin, int mVperAmp = ACS712_20A);
    ~ACS712();
    
    /**
     * Read the current current in amps
     */
    float read();
    
    /**
     * Read the current AC current in amps
     */
    float readAC();
    
    /**
     * Update the RMS calculation for AC current. Must be called at greater than 1.8kHz
     */
    void updateAC();

  private:
    float rolling_buf_[30];
    int buffer_index_ = 0;
    unsigned long last_sample_time_ = millis();

    int analog_pin_;
    int mVperAmp_;  
};

#endif /* _ACS_712_H_ */
