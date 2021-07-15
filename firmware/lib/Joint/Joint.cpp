#include "Joint.h"

inline float mapf(float value, float src_min, float src_max, float dest_min, float dest_max){
    return (value - src_min) * (dest_max - dest_min) / (src_max - src_min) + dest_min;
}

inline float clamp(float value, float max, float min){
    return value > max? max : (value < min? min : value);    
}

Joint::Joint(uint8_t pin, float inf_lim, float sup_lim){
    this->pin = pin;
    this->inf_lim = inf_lim;
    this->sup_lim = sup_lim;
    this->angle = 0.0f;
    this->max_speed = 5.0f; // 5 degrees/s    
    this->int_f = Interpolations::linear;
}

void Joint::init(){
    this->servo.attach(this->pin);        
}

void Joint::set_speed(float max_speed){
    this->max_speed = max_speed;
}

void Joint::set_position(float target_pos){
    int in_us;
    target_pos = clamp(target_pos, this->sup_lim, this->inf_lim);    
    this->angle = target_pos;
    in_us = (int) mapf(target_pos, -90.0f, 90.0f, MIN_SERVO_PULSE_WIDTH, MAX_SERVO_PULSE_WIDTH);
    this->servo.writeMicroseconds(in_us);
}

float Joint::get_position(){
    return this->angle;
}

bool Joint::reached_limit(){
    return ((this->angle == this->sup_lim) || (this->angle == this->inf_lim));
}

void Joint::set_interpolation_target(float target_pos){
    float delta_pos;

    this->int_starting_pos = this->angle;
    this->int_final_pos = target_pos;
    delta_pos = this->int_final_pos - this->int_starting_pos;
    if(delta_pos < 0)
        delta_pos *= -1;
    this->int_duration = (unsigned long) ((delta_pos/this->max_speed) * 1000.0f);
    // this->int_f = ifunc;
    this->int_starting_t = millis();
    this->int_final_t = this->int_starting_t + this->int_duration;    
}

void Joint::set_interpolation_function(float (*int_f)(float, float, float)){
    this->int_f = int_f;
}

bool Joint::interpolation_step(){
    float curr_i_angle, i_value;

    if(this->int_duration != 0){
        i_value = (float)(millis() - this->int_starting_t)/(float)this->int_duration;
        i_value = clamp(i_value, 1.0f, 0);
    }
    else{
        return true;
    }
    curr_i_angle = this->int_f(i_value, this->int_starting_pos, this->int_final_pos);
    this->set_position(curr_i_angle);
    return (i_value == 1.0f);
}