/**
 * 
 * 
 * 
 **/
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <NU32Hardware.h>

ros::NodeHandle_<NU32Hardware> nh;

////////////////
// Subscribers
////////////////
void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(PA_9, HIGH-digitalRead(PA_9));   // blink the led
}
ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


////////////////
// Publishers
////////////////


////////////////
// Services
////////////////

void setup() {
  //Initialise ros
  nh.initNode();
  //Subscribers
  //Publishers
  //Services
  nh.subscribe(sub);

  //Peripheral setup
  pinMode(PA_9, OUTPUT);
}

void loop() {
  //loop
  nh.spinOnce();
  delay(1);
}