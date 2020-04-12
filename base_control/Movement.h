//This class has all the functions related to Movement.
#ifndef MovementClass
#define MovementClass

enum direction{
    left,
    right
};

//MOVEMENT
class Movement {
  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    Movement(BNO * bno);


    //////////////////////////////////Motors//////////////////////////////////////
    Motor back_right_motor_;
    Motor back_left_motor_;
    Motor front_right_motor_;
    Motor front_left_motor_;


    //////////////////////////////////Encoders//////////////////////////////////////
    //Initialize motor encoders.
    void initEncoders();


    //////////////////////////////////PWM//////////////////////////////////////
    ///Set same pwm to all motors.
    void changePwm(int pwm);


    //////////////////////////////////VELOCITY//////////////////////////////////////
    ///Change deltaX value.
    void setDeltaX(double delta_x);
    ///Change deltaY value.
    void setDeltaY(double delta_y);
    ///Change deltaAngular value.
    void setDeltaAngular(double delta_angular);
    ///Return target linear angle.
    double getTargetAngle();
    ///Return target linear velocity.
    double getTargetLinearVelocity();
    ///Return target anuglar velocity.
    double getTargetAngularVelocity();
    //Stop robot.
    void stop();


    //////////////////////////////////DIRECTIONS//////////////////////////////////////
    ///This function tells where is the target Angle of the class,left or right.
    direction whereToGo(double &actual_angle);
    ///This function tells where is a custom target Angle,left or right.
    direction whereToGo(double &actual_angle,double target_angle);
    //This function change any angle to a direction angle. 
    int angleToDirection(int angle);
    ///This function sets a direction according to an angle.
    void setDirection(int angle); 
    ///Put motors in 0° direction.
    void move0();
    ///Put motors in 45° direction.
    void move45();
    ///Put motors in 90° direction.
    void move90();
    ///Put motors in 135° direction.
    void move135();
    ///Put motors in 180° direction.
    void move180();
    ///Put motors in 225° direction.
    void move225();
    ///Put motors in 270° direction.
    void move270();
    ///Put motors in 315° direction.
    void move315();
    ///Put motors rotating left.
    void rotateLeft();
    ///Put motors rotating right.
    void rotateRight();
 
    
    //////////////////////////////////PID//////////////////////////////////////
    ///Set motors to respect linear velocity. 
    void constantLinearSpeed();
    ///Set motors to respect angular velocity. 
    void constantAngularSpeed();
    ///Fix robot inclination while is moving.
    void velocityAdjustment(int adjustment);
    ///Robot moves linear with pid.
    void pidLinearMovement();
    ///Robot moves angular with pid.
    void pidAngularMovement();
    ///Robot rotate to a custom targetAngle whit pid.
    bool pidRotate(double target_angle);
    
  private:
    //Pins
    static const uint8_t kIdBackLeftMotor=1;
    static constexpr uint8_t kDigitalPinsBackLeftMotor[2]={15,14};
    static const uint8_t kAnalogPinBackLeftMotor=11;
    static constexpr uint8_t kEncoderPinsBackLeftMotor[2]={2,37};

    static const uint8_t kIdFrontLeftMotor=2;
    static constexpr uint8_t kDigitalPinsFrontLeftMotor[2]={37,36};
    static const uint8_t kAnalogPinFrontLeftMotor=5;
    static constexpr uint8_t kEncoderPinsFrontLeftMotor[2]={2,22};

    static const uint8_t kIdBackRightMotor=3;
    static constexpr uint8_t kDigitalPinsBackRightMotor[2]={45,43};
    static const uint8_t kAnalogPinBackRightMotor=6;
    static constexpr uint8_t kEncoderPinsBackRightMotor[2]={19,14};

    static const uint8_t kIdFrontRightMotor=4;
    static constexpr uint8_t kDigitalPinsFrontRightMotor[2]={25,24};
    static const uint8_t kAnalogPinFrontRightMotor=7;
    static constexpr uint8_t kEncoderPinsFrontRightMotor[2]={18,23};

    ///Bno
    BNO *bno_;
    
    ///Pid
    int target_angle_=0;
    double straight_output_=0;
    PID pid_straight_;
    PID pid_rotation_;
    static const uint8_t kPidMovementTimeSample=100;
    static const uint16_t kPidMaxErrorSum=3000;
    static constexpr double kOutputMinLimitPidStraight=-0.175;
    static constexpr double kOutputMaxLimitPidStraight=0.175;
    static constexpr double kPPidStraight=0.0008;
    static constexpr double kIPidStraight=0.00015;
    static constexpr double kDPidStraight=0.0002;
    static constexpr double kOutputMinLimitPidRotation=-0.18;
    static constexpr double kOutputMaxLimitPidRotation=0.18;
    static constexpr double kOutputAdjustment=-0.075;
    static constexpr double kPidRotationTolerance=1;
    static constexpr double kPPidRotation=0.00050;
    static constexpr double kIPidRotation=0.00110;
    static constexpr double kDPidRotation=0.00080;
    
    ///Velocity
    double delta_x_=0;
    double delta_y_=0;
    double delta_angular_=0;

    //Constants
    static const uint16_t kIntMax=65535;
    static const uint16_t kMinAngle=0;
    static const uint16_t kIntermediateAngle=180;
    static const uint16_t kMaxAngle=360;
    static const uint16_t kDirectionSeparationAngle=45;
    static const uint16_t kCountDirections=kMaxAngle/kDirectionSeparationAngle;
        
};
#endif
