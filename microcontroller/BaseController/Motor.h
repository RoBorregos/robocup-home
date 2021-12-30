// This class has all the functions related to one Motor.
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>

#include "MotorId.h"
#include "PID.h"

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
    // Calculate PWM equivalent to a target speed
    double approxSpeedToPWM(const double target_speed);
    
    // Change Speed of a motor. (PWM and Direction)
    void setMotorSpeed(const double pwm);
    
    // Change Speed of a motor using PID controller.
    void setMotorSpeedPID(const double speed);
    
    // Calculate target Rps according to a velocity in meters per second.
    double getTargetRps(const double velocity);

    // Calculate Revolutions per second equivalent to meters per second.
    double MsToRps(const double ms);
    
    // Calculate target Ticks according to a velocity  in meters per second.
    double getTargetTicks(const double velocity);
    
    // Change PWM of a motor, same direction.
    void changePwm(const uint8_t pwm);

    //////////////////////////////////Set Methods//////////////////////////////////////
    // Set the count of ticks of the encoders, the count used in Odometry.
    void setEncoderTicks(const long encoder_ticks);
    // Add a delta to the count of ticks of the encoders, the count used in Odometry.
    void deltaEncoderTicks(const int delta_encoder_ticks);

    // Set the count of ticks of the encoders, the count used in Pid.
    void setPidTicks(const int pid_ticks);
    // Add a delta to the count of ticks of the encoders, the count used in Pid.
    void deltaPidTicks(const int delta_pid_ticks);


    //////////////////////////////////Get Methods//////////////////////////////////////
    // Return the count of ticks of the encoders, the count used in Odometry.
    long getEncoderTicks();
    // Return the count of ticks of the encoders, the count used in Pid.
    int getPidTicks();

    // Return the current speed of the motor in meteres per second.
    double getCurrentSpeed();
    // Return the target speed of the motor in meteres per second.
    double getTargetSpeed();

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
  public:
    static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;
  private:  
    static constexpr double kPwmDeadZone = 50;
    static constexpr double kMinPwmForMovement = 120;
    static constexpr double kMaxPWM = 255;
    
    // Pins.
    uint8_t digital_one_;
    uint8_t digital_two_;
    uint8_t analog_one_;
    uint8_t encoder_one_;
    uint8_t encoder_two_;

    // Velocity.
    uint8_t pwm_ = 0;
    int pid_ticks_ = 0;
    long encoder_ticks_ = 0;
    double last_ticks_ = 0;
    double current_speed_ = 0;
    double target_speed_ = 0;

    // PID.
    PID pid_;
    static constexpr uint8_t kPidMinOutputLimit = 0;
    static constexpr uint8_t kPidMaxOutputLimit = 255;
    static constexpr uint16_t kPidMaxErrorSum = 4000;
    static constexpr uint8_t kPidMotorTimeSample = 100;
    static constexpr double kOneSecondInMillis = 1000.0;
    static constexpr double kSecondsInMinute = 60;
    static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorTimeSample;
    static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;
    static constexpr double kP = 45;
    static constexpr double kI = 55;
    static constexpr double kD = 40;
};

#endif
