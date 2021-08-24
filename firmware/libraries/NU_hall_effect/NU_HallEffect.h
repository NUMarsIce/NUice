#include <Arduino.h>
#include <NUros.h>
#include <std_msgs/Float64.h>
#include <NU_Driver.h>

class NUHallEffect: public NUDriver {
private:
  ros::Publisher pub;
  std_msgs::Float64 msg;
  int pin;
  long last_ping = millis(), last_pub = millis();
  const float time_out = 5000;
  const float debounce = 10;
  bool last_state = LOW;

public:
  NUHallEffect(ros::NUNodeHandle& nh, const char* ns, int pin): NUDriver(nh, ns), pub(appendNamespace("/rate"), &msg) {
    this->pin = pin;
  }

  void setup() {
    nh_.initNode();
    nh_.advertise(pub);
    pinMode(pin, INPUT);
  }

  void update() {
    // Zero timeout if its been more than a second
    if(millis() - last_ping > 1000)
        msg.data = 0;

    // Check if pulsed  
    bool current_state = digitalRead(pin);
    if(!current_state && last_state) {
        float ping = millis();
        float dt = ping - last_ping;
        if(dt > time_out || dt < debounce) {
            last_ping = ping;
            return;
        } 
        msg.data = 1000.0/dt;
        last_ping = ping;
    }
    last_state = current_state;
    
    //Publish data
    if(millis()-last_pub > 100){
        pub.publish(&msg);
        last_pub = millis();
    }
}     
};


