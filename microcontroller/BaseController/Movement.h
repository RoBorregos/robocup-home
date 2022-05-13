// This class has all the functions related to Movement.
#ifndef Movement_h
#define Movement_h

#include "MotorId.h"

#include <ros.h>
#include <math.h>
#include <Arduino.h>

#include "BNO.h"
#include "Kinematics.h"
#include "Motor.h"

#define PI 3.1415926535897932384626433832795

inline int sign(int a) { return min(1, max(-1, a)); };

class Movement {
  private:
  
    // Constants
    static constexpr uint8_t kCountMotors = 4;

  public:
  
    //////////////////////////////////Constructor//////////////////////////////////////
    Movement(BNO * bno, ros::NodeHandle *nh);


    //////////////////////////////////Motors//////////////////////////////////////
    Motor back_right_motor_;
    Motor back_left_motor_;
    Motor front_right_motor_;
    Motor front_left_motor_;

    //////////////////////////////////Encoders//////////////////////////////////////
    // Initialize motor encoders.
    void initEncoders();


    //////////////////////////////////PWM//////////////////////////////////////
    // Set same pwm to all motors.
    void changePwm(const uint8_t pwm);


    //////////////////////////////////VELOCITY//////////////////////////////////////    
    // Received a Velocity Command.
    void cmdVelocity(const double linear_x, const double linear_y, const double angular_z);

    // Calculate meters per second equivalent to Revolutions per minute.
    double RpmToMs(const double rpm);
    
    // Stop robot.
    void stop();

    // Get Encoder Counts in x, y, theta.
    void getEncoderCounts(int *delta_encoder_counts);
    // Pins,
    static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {15, 14};
    static constexpr uint8_t kAnalogPinFrontLeftMotor = 11;
    static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 25};

    static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {22, 23};
    static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
    static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 37};

    static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {26,27};
    static constexpr uint8_t kAnalogPinFrontRightMotor = 7;
    static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {19, 33};

    static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {30,31};
    static constexpr uint8_t kAnalogPinBackRightMotor = 8;
    static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {18, 29};

  private:
    // ROS
    ros::NodeHandle * nh_;

    // Bno.
    BNO *bno_;

    // Velocity limits based on Motor::kMaxVelocity and Mechanic limitations.
    static constexpr double kLinearXMaxVelocity = 0.20;
    static constexpr double kLinearYMaxVelocity = 0.20;
    static constexpr double kAngularZMaxVelocity = 0.20;

    // Encoders
    int last_encoder_counts_[Movement::kCountMotors];

    // Constants
    static constexpr bool kUsingPID = true;

    // Kinematics
    Kinematics kinematics_;
    static constexpr uint8_t MOTOR_MAX_RPM = 40;        // Motor's maximum rpm.
    static constexpr double WHEEL_DIAMETER = 0.1;       // Robot's wheel diameter expressed in meters.
    static constexpr double FR_WHEEL_DISTANCE = 0.50;   // Distance between front wheel and rear wheel.
    static constexpr double LR_WHEEL_DISTANCE = 0.40;   // Distance between left wheel and right wheel.
    static constexpr uint8_t PWM_BITS = 8;              // Microcontroller's PWM pin resolution.
    static constexpr uint8_t IS_OMNI = false;           // Omni Wheels
        
};

#endif
