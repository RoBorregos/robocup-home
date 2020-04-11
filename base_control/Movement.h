#ifndef MovementClass

#define MovementClass

#define MOVEMENT_TIME_VELOCITY_SAMPLE 100.0 //millis

enum direction{
    left,
    right
};

//MOVEMENT
class Movement {
  public:
    
    //BACK 
    Motor B_right;
    Motor B_left;
    //FRONT 
    Motor F_right;
    Motor F_left;
    
    Movement(BNO * bno);

    //Encoders
    void initEncoders();
    
    //PWM
    void pwm(int i);

    //VELOCITY
    double dX=0.127;
    double dY=0.127;
    double dteta=0;
    double getTargetLinearVelocity();
    double getTargetAngularVelocity();
    double getTargetAngle();
     
    
    //DIRECTIONS
    direction whereToGo(double &actualAngle);
    direction whereToGo(double &actualAngle,double _targetAngle);
    void setDirection(int angle); 
    void _move0();
    void _move45();
    void _move90();
    void _move135();
    void _move180();
    void _move225();
    void _move270();
    void _move315();
    
    //ROTATE
    void _rotateL();
    void _rotateR();
 
    //STOP
    void _stop();
    
    //PID
    int targetAngle=0;
    double straightOutput=0;
    PID _PIDStraight;
    PID _PIDRotation;
    void constantAngularSpeed();
    void constantLinearSpeed();
    void doVelocityAdjustment(int adjustment);
    void pidLinearMovement();
    void pidAngularMovement();
    bool pidRotate(double _targetAngle);

    //Utils
    int AngleToDirection(int angle);
    
  private:
    BNO *bno_;
};
#endif
