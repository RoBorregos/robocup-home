// This class has all the functions related to one Motor.
#ifndef Motor_h
#define Motor_h

enum class MotorState {
    Forward = 1, 
    Backward = 2, 
    Stop = 3
};

class Motor {
  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    Motor();
    Motor(const uint8_t id, const uint8_t digital_one, const uint8_t digital_two, const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two);
    
    
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
    // Calculate target Rpm according to a velocity.
    double getTargetRpm(const double velocity);
    // Calculate target Ticks according to a velocity.
    double getTargetTicks(const double velocity);
    // Change Pwm value of a motor.
    void changePwm(const uint8_t pwm);
    // Compute Pid controller and update pwm. 
    void constantSpeed(const double velocity);


    //////////////////////////////////Set Methods//////////////////////////////////////
    // Set the count of ticks of the encoders, the count used in Pid.
    void setPidTicks(const int pid_ticks);
    // Set the count of ticks of the encoders, the count used in Odometry.
    void setOdomTicks(const int odom_ticks);
    // Set an adjustment to the velocity.
    void setVelocityAdjustment(const double velocity_adjustment);    


    //////////////////////////////////Get Methods//////////////////////////////////////
    // Return the count of ticks of the encoders, the count used in Pid.
    int getPidTicks();
    // Return the count of ticks of the encoders, the count used in Odometry.
    int getOdomTicks();
    // Return the last count of ticks of the encoders, before it was reset in Pid process.
    double getLastTicks();
    // Return the current speed of the motor in meteres per second.
    double getCurrentSpeed();
    // Return the current state of the motor.
    MotorState getCurrentState();

  private:
    uint8_t id_ = 0;
    MotorState current_state_;
    
    // Motor Characteristics.
    static const uint16_t kPulsesPerRevolution = 4320;
    static constexpr double kWheelDiameter = 0.1;
    
    // Robot Movement Limits.
    static const uint16_t kMaxTicks = 286;
    static const uint16_t kMinTicks = 190;
    
    // Pins.
    uint8_t digital_one_;
    uint8_t digital_two_;
    uint8_t analog_one_;
    uint8_t encoder_one_;
    uint8_t encoder_two_;

    // Velocity.
    uint8_t pwm_ = 0;
    int pid_ticks_ = 0;
    int odom_ticks_ = 0;
    double last_ticks_ = 0;
    double current_speed_ = 0;
    double velocity_adjustment_ = 0;
    
    // PID.
    PID pid_;
    static const uint8_t kPidMinOutputLimit = 0;
    static const uint8_t kPidMaxOutputLimit = 255;
    static const uint16_t kPidMaxErrorSum = 4000;
    static const uint8_t kPidMotorTimeSample = 100;
    static const uint16_t kOneSecondInMillis = 1000;
    static const uint16_t kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorTimeSample;
    static constexpr double kP = 45;
    static constexpr double kI = 55;
    static constexpr double kD = 40;

    // Constants.
    static const uint8_t kIdBackLeftMotor = 1;
    static const uint8_t kIdFrontLeftMotor = 2;
    static const uint8_t kIdBackRightMotor = 3;
    static const uint8_t kIdFrontRightMotor = 4;

};


#endif
