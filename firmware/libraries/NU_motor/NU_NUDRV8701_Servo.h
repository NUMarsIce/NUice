#include <Arduino.h>
#include <NUros.h>
#include <NU_Driver.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include "QuadratureEncoder.h"

/**
 * Driver for the DC motor driver on the movement board.
 */
class NUDRV8701Servo : public NUDriver {

public:
    /**
     * Constructs the driver based based on what number driver it is on the movement board which has dedicated pins for each driver
     */
    NUDRV8701Servo(ros::NUNodeHandle& nh, const char* ns, uint8_t movement_board_number, uint8_t encoder_a, uint8_t encoder_b, uint8_t max_speed = 100, uint8_t update_hz = 4);

    NUDRV8701Servo(ros::NUNodeHandle& nh, const char* ns, uint8_t cur_pin, uint8_t ph_pin, uint8_t en_pin, uint8_t nsleep_pin, uint8_t nfault_pin, uint8_t encoder_a, uint8_t encoder_b, uint8_t max_speed = 100, uint8_t update_hz = 4);

    void setup();

    void update();

private:
    uint8_t cur_pin_, ph_pin_, en_pin_, nsleep_pin_, nfault_pin_, enc_a_, enc_b_;
    unsigned long last_update_ = millis(), last_logic_update_ = millis();
    uint8_t update_hz_, max_speed_;
    int16_t target_pos_ = 0;
    const float p_const_ = 20.0; // for p controller 

    Encoders* encoder_;

    //Subscribers
    ros::Subscriber<std_msgs::Bool, NUDRV8701Servo> en_sub;
    void enable_cb(const std_msgs::Bool& msg){
        digitalWrite(nsleep_pin_, msg.data);
    }

    ros::Subscriber<std_msgs::UInt8, NUDRV8701Servo> speed_sub;
    void speed_cb(const std_msgs::UInt8& msg){
        max_speed_ = msg.data;
    }

    ros::Subscriber<std_msgs::Int32, NUDRV8701Servo> pos_sub;
    void pos_cb(const std_msgs::Int32& msg){
        target_pos_ = msg.data;
    }

    ros::Subscriber<std_msgs::Empty, NUDRV8701Servo> zero_sub;
    void zero_cb(const std_msgs::Empty& msg){
        encoder_->setEncoderCount(0);
        target_pos_ = 0;
    }

    //Publishers
    std_msgs::Bool fault_pub_msg_;
    ros::Publisher fault_pub;

    std_msgs::Float32 current_pub_msg_;
    ros::Publisher current_pub;

    std_msgs::Int32 pos_pub_msg_;
    ros::Publisher pos_pub;

    std_msgs::UInt32 error_pub_msg_;
    ros::Publisher error_pub;

};