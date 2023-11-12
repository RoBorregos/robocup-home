// This namespace contains the handler functions of the Encoders.
#ifndef Encoder_h
#define Encoder_h

#include "Motor.h"

namespace Encoder{
    //////////////////////////////////Main Function//////////////////////////////////////
    // Count ticks for specific motor. Sign Variable to indicate direction.
    void handleEncoder(Motor &motor, int sign);


    //////////////////////////////////Motor Functions//////////////////////////////////////
    
    // Call main function for back left motor.
    void backLeftEncoder();
    
    // Call main function for back right motor.
    void backRightEncoder();
    
    // Call main function for front left motor.
    void frontLeftEncoder();
    
    // Call main function for front right motor.
    void frontRightEncoder();
    
};
#endif