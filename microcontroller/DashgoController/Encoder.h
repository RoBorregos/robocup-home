// This namespace contains the handler functions of the Encoders.
#ifndef Encoder_h
#define Encoder_h

#include "Motor.h"

namespace Encoder{
    //////////////////////////////////Main Function//////////////////////////////////////
    // Count ticks for specific motor.
    void handleEncoder(Motor &motor, int sign);


    //////////////////////////////////Motor Functions//////////////////////////////////////
    
    // Call main function for left motor.
    void leftEncoder();
    
    // Call main function for right motor.
    void rightEncoder();
    
};
#endif