// This class has all the functions related to Movement.
#ifndef Movement_h
#define Movement_h

#include "MotorId.h"

#include "Kinematics.h"
#include <math.h>
#include <Arduino.h>

#include "Motor.h"

enum class Direction{
    left = 1,
    right = 2 
};

class Movement {
  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    Movement();


    //////////////////////////////////Motors//////////////////////////////////////
    Motor back_right_motor_;
    Motor back_left_motor_;
    Motor front_right_motor_;
    Motor front_left_motor_;
    int kMotorCount = 4;

    //////////////////////////////////Encoders//////////////////////////////////////
    // Initialize motor encoders.
    void initEncoders();


    //////////////////////////////////PWM//////////////////////////////////////
    // Set same pwm to all motors.
    void changePwm(const uint8_t pwm);


    //////////////////////////////////VELOCITY//////////////////////////////////////
    // Change deltaX value.
    void setDeltaX(const double delta_x);
    
    // Change deltaY value.
    void setDeltaY(const double delta_y);
    
    // Change deltaAngular value.
    void setDeltaAngular(const double delta_angular);

    // Return deltaX value.
    double getDeltaX();
    
    // Return deltaY value.
    double getDeltaY();
    
    // Return deltaAngular value.
    double getDeltaAngular();
    
    // Stop robot.
    void stop();
    
    //////////////////////////////////PID//////////////////////////////////////
    // Robot linear velocity to rpm per motor. 
    void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z);
    
    // Set motors to velocity. 
    void updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed);

  private:
  
    //////////////////////////////////DIRECTIONS//////////////////////////////////////
    // Convert radians to degrees.
    double radiansToDegrees(const double radians);

    // Pins,
    static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {15, 14};
    static constexpr uint8_t kAnalogPinFrontLeftMotor = 11;
    static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {3, 49};

    static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {23, 22};
    static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
    static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {2, 48};

    static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {26,27};
    static constexpr uint8_t kAnalogPinFrontRightMotor = 7;
    static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {19, 45};

    static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {30,31};
    static constexpr uint8_t kAnalogPinBackRightMotor = 8;
    static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {18, 44};

    // Velocity maximum.
    static constexpr double kFrWheelsDist = 0.0;
    static constexpr double kLrWheelsDist = 0.33;
    static constexpr double kWheelDiameter = 0.12;
    static constexpr double kRPM = 120;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * (M_PI * kWheelDiameter);

    static constexpr double kLinearXMaxVelocity = kMaxVelocity;
    static constexpr double kLinearYMaxVelocity = kMaxVelocity;
    static constexpr double kAngularZMaxVelocity = kMaxVelocity / kLrWheelsDist;
    static constexpr uint8_t kPwmBits = 8;
    long long cycle = 0;

    // Kinematics.
    Kinematics kinematics_;

    // Velocity.
    double delta_x_ = 0;
    double delta_y_ = 0;
    double delta_angular_ = 0;
};
#endif
