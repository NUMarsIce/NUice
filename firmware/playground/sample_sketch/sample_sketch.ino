/**
 * 
 * Simple sketch that subscribes to the "toggle_led" topic and flashes PA_9 LED.
 * To flash the led run `rostopic pub /toggle_led std_msgs/Empty -r 30` 
 **/
#define USE_USB

#ifndef __STM32f4__
  #define PA_9 13
#endif

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

// LED Toggle Subscriber
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(PA_9, HIGH-digitalRead(PA_9));   // blink the led
}
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


void setup() {
    nh.initNode();
    nh.subscribe(sub);
    pinMode(PA_9, OUTPUT);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
