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
    // Motor Characteristics.
    static constexpr double kPulsesPerRevolution = 4320.0;
    static constexpr double kWheelDiameter = 0.105;
    static constexpr double kWheelCircumference = 0.33;
    
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
    // Calculate target Rps according to a velocity in meters per second.
    void RpmToPwm(const double rpm);

    // Change Pwm value of a motor.
    void changePwm(const uint8_t pwm);
    
    // Compute Pid controller and update pwm. 
    void stableRPM(const double velocity);


    //////////////////////////////////Set Methods//////////////////////////////////////
    // Set the count of ticks of the encoders, the count used in Pid.
    void setPidTicks(const int pid_ticks);
    
    // Set the count of ticks of the encoders, the count used in Odometry.
    void setOdomTicks(const int odom_ticks);
    
    // Set the direction of the wheels according to encoders.
    void setEncodersDir(const int encoders_dir);

    //////////////////////////////////Get Methods//////////////////////////////////////
    // Get the direction of the wheels according to encoders.
    int getEncodersDir();

    // Return the count of ticks of the encoders, the count used in Pid.
    int getPidTicks();
    
    // Return the count of ticks of the encoders, the count used in Odometry.
    int getOdomTicks();
    
    // Return the last count of ticks of the encoders, before it was reset in Pid process.
    double getLastTicks();
    
    // Return the target Speed of the motor in meters per second.
    double getTargetSpeed();

    // Return the current speed of the motor in meteres per second.
    double getCurrentSpeed();
    
    // Return the current state of the motor.
    MotorState getCurrentState();

    // Get Encoder One pin.
    uint8_t getEncoderOne();

    // Get Encoder Two pin.
    uint8_t getEncoderTwo();

    // Get PWM
    uint8_t getPWM();


  private:
    MotorId id_;
    MotorState current_state_ = MotorState::Forward;
    
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
    double tmp_pwm = 0;
    int encoders_dir_ = 0;
    int pid_ticks_ = 0;
    int odom_ticks_ = 0;
    double last_ticks_ = 0;
    double current_speed_ = 0;
    double target_speed_ = 0;
    double kMaxRpm = 100.0;
    double prevMillis = 0;
    double currentMillis = 0;
    double interval = 100;
    
    // PID.
    PID pid_;
    //////No olvidar cambiar de nuevo la salida de PWM mínima cuando termine
    static constexpr uint8_t kPidMinOutputLimit = 50;
    static constexpr uint8_t kPidMaxOutputLimit = 255;
    static constexpr uint32_t kPidMaxErrorSum = 100;
    static constexpr uint16_t kPidSampleTime = 100;
    static constexpr float kPidMotorSampleTime = 0.1;
    static constexpr double kOneSecondInMillis = 1000.0;
    static constexpr double kSecondsInMinute = 60;
    static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorSampleTime;
    static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;
    static constexpr double kP = 5.0;
    static constexpr double kI = 1.5;
    static constexpr double kD = 0;
    

};


#endif
