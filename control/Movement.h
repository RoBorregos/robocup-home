#ifndef MovementClass

#define MovementClass

//MOVEMENT
class Movement {
  public:
    
    //BACK MOTORS
    Motor B_right;
    Motor B_left;
    
    //FRONT MOTORS
    Motor F_right;
    Motor F_left;
    
    Movement(Motor A, Motor B, Motor C, Motor D);
    Movement();
    
    //DIRECTIONS
    void _move0();
    void _move45();
    void _move90();
    void _move135();
    void _move180();
    void _move225();
    void _move270();
    void _move315();
    void _rotateL();
    void _rotateR();
    void _stop();
    


    void pwm(int i);
    void stop1();

};
#endif