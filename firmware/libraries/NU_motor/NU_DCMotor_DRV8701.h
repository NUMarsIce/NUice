#include <Arduino.h>
#include <NUros.h>
#include <NU_Driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>


#define CUR_PIN  PA6
#define PH_PIN  PB10
#define EN_PIN  PB13
#define NSLP_PIN PC7
#define NFLT_PIN PA8

/**
 * Driver for the DC motor driver on the movement board.
 */
class DRV8701Driver : public NUDriver {

public:
    /**
     * Constructs the driver based based on what number driver it is on the movement board which has dedicated pins for each driver
     */
    DRV8701Driver(ros::NUNodeHandle& nh, const char* ns, uint8_t movement_board_number) :
                NUDriver(nh, ns),
                dir_sub(appendNamespace("/set_dir"), &DRV8701Driver::dir_cb, this),
                speed_sub(appendNamespace("/set_speed"), &DRV8701Driver::speed_cb, this),
                speed_sub(appendNamespace("/enable"), &DRV8701Driver::enable_cb, this),
                current_pub(appendNamespace("/current"), &current_pub_msg_),
                fault_pub(appendNamespace("/fault"), &fault_pub_msg_)
    {
        switch(movement_board_number){
            case 3: //PC4, PB0, PB1, PC8, PA10
                cur_pin_ = PC4;
                ph_pin_ = PB0;
                en_pin_ = PB1;
                nsleep_pin_ = PC8;
                nfault_pin_ = PA10;
                break;
            case 2: //PA7, PB15, PB14, PC6, PC9
                cur_pin_ = PA7;
                ph_pin_ = PB15;
                en_pin_ = PB14;
                nsleep_pin_ = PC6;
                nfault_pin_ = PC9;
                break;
            default; //case 0   PA6, PB10, PB13, PC7, PA8
                cur_pin_ = PA6;
                ph_pin_ = PB10;
                en_pin_ = PB13;
                nsleep_pin_ = PC7;
                nfault_pin_ = PA8;
                break;
        }
    }

    DRV8701Driver(ros::NUNodeHandle& nh, const char* ns, uint8_t cur_pin, uint8_t ph_pin, uint8_t en_pin, uint8_t nsleep_pin, uint8_t nfault_pin, uint8_t update_hz = 4) : 
                NUDriver(nh, ns),
                dir_sub(appendNamespace("/set_dir"), &DRV8701Driver::dir_cb, this),
                speed_sub(appendNamespace("/set_speed"), &DRV8701Driver::speed_cb, this),
                speed_sub(appendNamespace("/enable"), &DRV8701Driver::enable_cb, this),
                current_pub(appendNamespace("/current"), &current_pub_msg_),
                fault_pub(appendNamespace("/fault"), &fault_pub_msg_),
                cur_pin_(cur_pin), ph_pin_(ph_pin), en_pin_(en_pin), nsleep_pin_(nsleep_pin), nfault_pin_(nfault_pin)
    {}

    void setup() {
        pinMode(cur_pin_, INPUT);
        pinMode(ph_pin_, OUTPUT);
        pinMode(en_pin_, OUTPUT);
        pinMode(nsleep_pin_, OUTPUT);
        pinMode(nfault_pin_, INPUT);

        nh_.subscribe(dir_sub);
        nh_.subscribe(speed_sub);

        nh_.advertise(current_pub);
        nh_.advertise(fault_pub);
    }

    void update() {
        if(millis()-last_update_ > update_hz_){
            current_pub_msg_.data = (float)analogRead(cur_pin_)/1023*3.3/20/0.01; // Current reads 20x the voltage across the 0.01ohm resistor. 
            current_pub.publish(current_pub_msg_);

            fault_pub_msg_.data = digitalRead(nfault_pin_);
            fault_pub.publish(fault_pub_msg_);

            last_update_ = millis();
        }
    }


private:
    uint8_t cur_pin_, ph_pin_, en_pin_, nsleep_pin_, nfault_pin_;
    unsigned long last_update_ = millis();
    uint8_t update_hz_;

    //Subscribers
    ros::Subscriber<std_msgs::Bool, DRV8701Driver> dir_sub;
    void dir_cb(const std_msgs::Bool& msg){
        digitalWrite(ph_pin_, msg.data);
    }

    ros::Subscriber<std_msgs::Bool, DRV8701Driver> en_sub;
    void enable_cb(const std_msgs::Bool& msg){
        digitalWrite(en_pin_, msg.data);
    }

    ros::Subscriber<std_msgs::UInt8, DRV8701Driver> speed_sub;
    void speed_cb(const std_msgs::UInt8& msg){
        analogWrite(en_pin_, msg.data);
    }

    //Publishers
    std_msgs::Bool fault_pub_msg_;
    ros::Publisher fault_pub;

    std_msgs::Float32 current_pub_msg_;
    ros::Publisher current_pub;
};