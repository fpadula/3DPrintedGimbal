#ifndef _JOINT_H_
#define _JOINT_H_

#include <Arduino.h>
#include <Servo.h>
#include <Interpolations.h>

#define MIN_SERVO_PULSE_WIDTH 544.0f
#define MAX_SERVO_PULSE_WIDTH 2400.0f

class Joint{
    private:                 
        float angle, max_speed, inf_lim, sup_lim, int_final_pos, int_starting_pos; 
        float (*int_f)(float, float, float);
        unsigned long int_duration, int_starting_t, int_final_t;
        uint8_t pin;
        Servo servo;
    public:
        Joint(uint8_t, float, float);        
        void set_position(float);
        void set_speed(float);
        void set_interpolation_target(float);
        void set_interpolation_function(float (*)(float, float, float));
        bool interpolation_step();
        float get_position();
        bool reached_limit();
        void init();
};


#endif