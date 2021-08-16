#include "ACS712.h"

ACS712::ACS712(int pin, int mVperAmp) { 
  analog_pin_ = pin; 
  mVperAmp_ = mVperAmp;
}

ACS712::~ACS712(){}

float ACS712::read() {
  float raw_value = analogRead(analog_pin_);
  float voltage = (raw_value / 1024.0) * 5000;  // Gets you mV
  return ((voltage - 2500) / mVperAmp_);
}

float ACS712::readAC() {
  // Find the mean of squares
  float sqsum = 0;
  for (int i = 0; i < 30; i++) {
    sqsum = sqsum + (pow(rolling_buf_[i], 2));
  }

  // find the square root of that mean
  return sqrt(sqsum / 30);
}

void ACS712::updateAC(){
  if(millis()-last_sample_time_ < 1/1800)//force the update to happen at ~1800Hz (30*60 aka 30 samples per 60Hz AC period)
    return;

  float raw_value = analogRead(analog_pin_);
  float voltage = (raw_value / 1024.0) * 5000;  // Gets you mV
  float sample_current = ((voltage - 2500) / mVperAmp_);

  // Update the buffer
  rolling_buf_[buffer_index_] = sample_current;

  // update the roll index
  if (buffer_index_ >= (29)) {
    buffer_index_ = 0;
  } else {
    buffer_index_++;
  }

  //update sample time
  last_sample_time_ = millis();
}