// This class has all the functions related to Movement.
#ifndef Movement_h
#define Movement_h

#include "MotorId.h"

#include <ros.h>
#include <math.h>
#include <Arduino.h>

#include "BNO.h"
#include "Motor.h"

inline int sign(int a) { return min(1, max(-1, a)); };

class Movement {
  private:
  
    // Constants
    static constexpr double kMaxPWM = 255;
    static constexpr uint8_t kCountMotors = 4;
    static constexpr uint16_t kIntMax = 65535;
    static constexpr uint16_t kCountReset = 250;
    static constexpr uint16_t kCountOverflow = 16374;

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
    // Change Range from Velocity to PWM.
    double convertToPWM(float value, const float maxValue);
    
    // Received a Velocity Command.
    void cmdVelocity(const double linear_x, const double linear_y, const double angular_z);
    
    // Stop robot.
    void stop();

    // Get Encoder Counts in x, y, theta.
    void getEncoderCounts(float *xytCounts);
    
  private:

    // Pins,
    static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {14, 15};
    static constexpr uint8_t kAnalogPinFrontLeftMotor = 11;
    static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {3, 25};

    static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {22, 23};
    static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
    static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {2, 37};

    static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {27,26};
    static constexpr uint8_t kAnalogPinFrontRightMotor = 7;
    static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {19, 33};

    static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {30,31};
    static constexpr uint8_t kAnalogPinBackRightMotor = 8;
    static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {18, 29};

    // ROS
    ros::NodeHandle * nh_;

    // Bno.
    BNO *bno_;

    // Velocity limits based on Motor::kMaxVelocity and Mechanic limitations.
    static constexpr double kDeadZone = 100;
    static constexpr double kLinearXMaxVelocity = 0.19;
    static constexpr double kLinearYMaxVelocity = 0.19;
    static constexpr double kAngularZMaxVelocity = 0.19;

    // Encoders
    int last_encoder_counts_[Movement::kCountMotors];
        
};

#endif
