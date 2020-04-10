#ifndef MotorClass
#define MotorClass

#define MOTOR_TIME_VELOCITY_SAMPLE 100.0 //millis
#define PULSES_PER_REVOLUTION 4320.0
#define WHEEL_DIAMETER 0.1 //meters
#define MINPWM 100
#define MAXTICKS 286
#define MINTICKS 190

enum motorState{
    forward,
    backward,
    stop
};

class Motor {
  public:
    byte id=0;

    double pwm = 0;
    int ticks=0;
    int odomTicks=0;
    double lastticks=0;
    double velocityAdjustment=0;

    //Pins
    byte d1 = 0;
    byte d2 = 0;
    byte p1 = 0;

    byte e1 = 0;
    byte e2 = 0;

    //STATE
    motorState actualState;
    
    //PID
    PID _PID;
    double kp = 45;
    double kd = 40;
    double ki = 55;
    double speedActual=0;
    double getTargetTicks(double velocity);
    double getTargetLinearTicks();
    double getTargetAngularTicks();
    void constantLinearSpeed();
    void constantAngularSpeed();



    Motor(byte id,byte d1, byte d2, byte p1, byte e1,byte e2);
    Motor();
    void Forward();
    void Backward();
    void Stop();
    void defineOutput();
    void changePWM(double p);
    int getTicks();
    int getOdomTicks();
    void setTicks(int ticks);
    double getTargetLinearRPM();
    double getTargetAngularRPM();
    double _OPID();
    
};


#endif
