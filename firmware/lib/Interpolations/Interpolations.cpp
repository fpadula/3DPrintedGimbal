#include "Interpolations.h"

float Interpolations::linear(float t, float start, float end){    
    return ((end - start)*t + start);
}

float Interpolations::expo(float t, float start, float end){    
    // float Easings::expo(float t, float start, float change, float duration){
    float change = (end - start);
    t = t * 2;

    if (t < 1) 
    {
        return change * 0.5 * pow(2, 10 * (t - 1)) + start;
    }

    t -= 1;
    return change * 0.5 * (-pow(2, -10 * t) + 2) + start;    
}

float Interpolations::bounce_out(float t, float start, float end){

    float change = (end - start);

    if (t < (1 / 2.75))
    {
        return change * (7.5625 * t * t) + start;
    }
    else
    if (t < (2 / 2.75))
    {
        t -= (1.5 / 2.75);
        return change * (7.5625 * t * t + 0.75) + start;
    }
    else
    if (t < (2.5 / 2.75))
    {
        t -= (2.25 / 2.75);
        return change * (7.5625 * t * t + 0.9375) + start;
    }
    else
    {
        t -= ( 2.625 / 2.75 );
        return change * (7.5625 * t * t + 0.984375) + start;
    } 
}

float Interpolations::inout_cubic(float t, float start, float end){
    // float Easings::bounce_out(float t, float start, float change, float duration){  
    float change = (end - start);
    t = t * 2;    

    if (t < 1)
    {
        return (change * 0.5) * pow(t, 3) + start;
    }
        
    return (change * 0.5) * (pow(t - 2, 3) + 2) + start;
}