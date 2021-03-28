#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <NU_Driver.h>

Class HallEffectDriver: public NUDriver {
  private:
  ros::Publisher pub;
  std_msgs::Float64 msg;
  int pin;
  float last_ping;
  float time_out;

  HallEffectDriver(int pin, std::string topic_name): NUDriver(ros::NodeHandle_<NU32Hardware>(), "hall_effect/") {
    pub = ros::Publisher(topic_name, &msg);
    this->pin = pin;
  }

  void setup() {
    nh.initNode();
    nh.advertise(pub);
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), publish, RISING);
    time_out = 5000;
    last_ping = millis();
  }

  void update() {
    nh.spinOnce();
  }

  void publish() {
    float ping = millis();
    float dt = ping - last_ping;
    if(dt > time_out) {
      return;
    } 
    msg.data = 1.0/dt;
    pub.publish(&msg);
    last_ping = ping;
  }

};
