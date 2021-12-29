// This class has all the functions related to one Motor.
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

#include "MotorId.h"

enum class MotorState {
    Forward = 1, 
    Backward = 2, 
    Stop = 3
};

class Motor {
  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    Motor();
    Motor(const MotorId id, const uint8_t digital_one, const uint8_t digital_two, 
    const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two);
    
    
    //////////////////////////////////Initialization//////////////////////////////////////
    // Declare motor pins as output.
    void defineOutput();
    
    // Declare encoder pins as input and include attachInterrupt.
    void initEncoders();
    

    //////////////////////////////////Motor State//////////////////////////////////////
    // Change motor state to forward.
    void forward();
    
    // Change motor state to backward.
    void backward();
    
    // Change motor state to stop.
    void stop();


    //////////////////////////////////Velocity//////////////////////////////////////
    // Change Speed of a motor. (PWM and Direction)
    void setMotorSpeed(const double pwm);

    //////////////////////////////////Set Methods//////////////////////////////////////
    // Set the count of ticks of the encoders, the count used in Odometry.
    void setEncoderTicks(const int encoder_ticks);


    //////////////////////////////////Get Methods//////////////////////////////////////
    // Return the count of ticks of the encoders, the count used in Odometry.
    int getEncoderTicks();
    
    // Return the current state of the motor.
    MotorState getCurrentState();

  private:
    MotorId id_;
    MotorState current_state_ = MotorState::Stop;
    
    // Motor Characteristics.
    static constexpr double kPulsesPerRevolution = 4320.0;
    static constexpr double kWheelDiameter = 0.1;
    static constexpr double kRPM = 40;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;

    // Robot Movement Limits.
    static constexpr uint16_t kMaxTicks = 286;
    static constexpr uint16_t kMinTicks = 190;
    
    // Pins.
    uint8_t digital_one_;
    uint8_t digital_two_;
    uint8_t analog_one_;
    uint8_t encoder_one_;
    uint8_t encoder_two_;

    // Velocity.
    uint8_t pwm_ = 0;
    int encoder_ticks_ = 0;

};

#endif
