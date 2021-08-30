#include <NU_NUDRV8701_Servo.h>

NUDRV8701Servo::NUDRV8701Servo(ros::NUNodeHandle& nh, const char* ns, uint8_t movement_board_number, uint8_t encoder_a, uint8_t encoder_b, uint8_t max_speed, uint8_t update_hz) :
            NUDriver(nh, ns),
            pos_sub(appendNamespace("/set_pos"), &NUDRV8701Servo::pos_cb, this),
            speed_sub(appendNamespace("/set_speed"), &NUDRV8701Servo::speed_cb, this),
            en_sub(appendNamespace("/enable"), &NUDRV8701Servo::enable_cb, this),
            zero_sub(appendNamespace("/zero"), &NUDRV8701Servo::zero_cb, this),    
            current_pub(appendNamespace("/current"), &current_pub_msg_),
            fault_pub(appendNamespace("/fault"), &fault_pub_msg_),
            pos_pub(appendNamespace("/current_position"), &pos_pub_msg_),
            error_pub(appendNamespace("/enc_errors"), &error_pub_msg_),
            max_speed_(max_speed),
            enc_a_(encoder_a), enc_b_(encoder_b),
            update_hz_(update_hz)
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
        default: //case 1   PA6, PB10, PB13, PC7, PA8
            cur_pin_ = PA6;
            ph_pin_ = PB10;
            en_pin_ = PB13;
            nsleep_pin_ = PC7;
            nfault_pin_ = PA8;
            break;
    }
}
NUDRV8701Servo::NUDRV8701Servo(ros::NUNodeHandle& nh, const char* ns, uint8_t cur_pin, uint8_t ph_pin, uint8_t en_pin, uint8_t nsleep_pin, uint8_t nfault_pin, uint8_t encoder_a, uint8_t encoder_b, uint8_t max_speed, uint8_t update_hz) : 
            NUDriver(nh, ns),
            pos_sub(appendNamespace("/set_pos"), &NUDRV8701Servo::pos_cb, this),
            speed_sub(appendNamespace("/set_speed"), &NUDRV8701Servo::speed_cb, this),
            en_sub(appendNamespace("/enable"), &NUDRV8701Servo::enable_cb, this),
            zero_sub(appendNamespace("/zero"), &NUDRV8701Servo::zero_cb, this),
            current_pub(appendNamespace("/current"), &current_pub_msg_),
            fault_pub(appendNamespace("/fault"), &fault_pub_msg_),
            pos_pub(appendNamespace("/current_position"), &pos_pub_msg_),
            error_pub(appendNamespace("/enc_errors"), &error_pub_msg_),
            max_speed_(max_speed),
            cur_pin_(cur_pin), ph_pin_(ph_pin), en_pin_(en_pin), nsleep_pin_(nsleep_pin), nfault_pin_(nfault_pin), enc_a_(encoder_a), enc_b_(encoder_b),
            update_hz_(update_hz)
{}

void NUDRV8701Servo::setup(){
        encoder_ = new Encoders(enc_a_, enc_b_);//apparently we cant do this in the constructor rip

        pinMode(cur_pin_, INPUT);
        pinMode(ph_pin_, OUTPUT);
        pinMode(en_pin_, OUTPUT);
        pinMode(nsleep_pin_, OUTPUT);
        pinMode(nfault_pin_, INPUT);

        nh_.subscribe(pos_sub);
        nh_.subscribe(speed_sub);
        nh_.subscribe(en_sub);
        nh_.subscribe(zero_sub);

        nh_.advertise(current_pub);
        nh_.advertise(fault_pub);
        nh_.advertise(pos_pub);
        nh_.advertise(error_pub);
}

void NUDRV8701Servo::update() {
    // servoing logic
    if(millis()-last_logic_update_ > 10){// 100Hz logic loop

        if(encoder_->getEncoderCount() < target_pos_) // direction
            digitalWrite(ph_pin_, HIGH);
        else
            digitalWrite(ph_pin_, LOW);

        if(abs(encoder_->getEncoderCount()-target_pos_) < 1) // speed
            analogWrite(en_pin_, 0);
        else
            analogWrite(en_pin_, (int)constrain( abs(encoder_->getEncoderCount()-target_pos_)*p_const_, 0, max_speed_ ));

    }

    // delayed publishing
    if(millis()-last_update_ > 1000.0f/update_hz_){
        current_pub_msg_.data = (float)analogRead(cur_pin_)/1023*3.3/20/0.01; // Current reads 20x the voltage across the 0.01ohm resistor. 
        current_pub.publish(&current_pub_msg_);

        fault_pub_msg_.data = digitalRead(nfault_pin_);
        fault_pub.publish(&fault_pub_msg_);
        
        pos_pub_msg_.data = encoder_->getEncoderCount();
        pos_pub.publish(&pos_pub_msg_);

        error_pub_msg_.data = encoder_->getEncoderErrorCount();
        error_pub.publish(&pos_pub_msg_);

        last_update_ = millis();
    }
}

