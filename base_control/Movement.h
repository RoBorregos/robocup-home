#ifndef MovementClass

#define MovementClass

//MOVEMENT
class Movement {
  public:
    
    //BACK 
    Motor B_right;
    Motor B_left;
    //FRONT 
    Motor F_right;
    Motor F_left;
    
    Movement(Motor A, Motor B, Motor C, Motor D);
    Movement();
    
    //PWM
    void pwm(int i);

    //DISTANCE
    double getVelocity(Motor A);

    //VELOCITY
    void calcVelocity();
    void calcVelocityaux(Motor &A);

    //INPUTS
    //test .18m/s 0°
    double dX=0.127;
    double dY=0.127;
    double angle=0;
    double getTargetVelocity();
    double getTargetTicks();
    double getTargetAngle();
    double getOrientationA();
     
    
    //DIRECTIONS
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
    void constantSpeed();

};
#endif
