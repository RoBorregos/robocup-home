#ifndef MotorClass

#define MotorClass
class Motor {
  public:
    byte id=0;

    double pwm = 0;
    double ticks=0;
    double lastticks=0;
    double velocity=0; // m/s
    unsigned long VelocityTiming = 0;

    //Pins
    byte d1 = 0;
    byte d2 = 0;
    byte p1 = 0;

    byte e1 = 0;
    byte e2 = 0;

    //STATE
    byte state = 0;

    //PID
    PID _PID;
    double kp = 50;
    double kd = 40;
    double ki = 55;
    double speedActual=0;
    void constantSpeed();


    Motor(byte id,byte d1, byte d2, byte p1, byte e1,byte e2);
    Motor();
    void Forward();
    void Backward();
    void Stop();
    void defineOutput();
    void changePWM(double p);
    int getTicks();
    void setTicks(int ticks);
    double getTargetSpeed();
    double _OPID();
    
};


#endif
