#ifndef Encoder_h
#define Encoder_h
///This namespace contains the handler functions of the Encoders.
namespace Encoder{
    //////////////////////////////////Main Function//////////////////////////////////////
    ///Count ticks for specific motor.
    void handleEncoder(Motor &motor);


    //////////////////////////////////Motor Functions//////////////////////////////////////
    ///Call main function for back left motor.
    void backLeftEncoder();
    ///Call main function for back right motor.
    void backRightEncoder();
    ///Call main function for front left motor.
    void frontLeftEncoder();
    ///Call main function for front right motor.
    void frontRightEncoder();
};
#endif