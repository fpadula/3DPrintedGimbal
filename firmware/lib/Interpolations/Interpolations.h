#ifndef _INTERPOLATIONS_H_
#define _INTERPOLATIONS_H_

#include <Arduino.h>

class Interpolations{
    public:
        static float linear(float, float, float);        
        static float expo(float, float, float);        
        static float bounce_out(float, float, float);        
        static float inout_cubic(float, float, float);        
};

#endif