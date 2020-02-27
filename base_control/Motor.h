#ifndef MotorClass

#define MotorClass
class Motor {
  public:
    byte id=0;

    byte pwm = 0;
    double ticks=0;

    //Pins
    byte d1 = 0;
    byte d2 = 0;
    byte p1 = 0;

    byte e1 = 0;
    byte e2 = 0;

    //STATE
    byte state = 0;

    //PID
    double Kp = 5.2, Ki = 0, Kd = 0;
    PID _PID;

    Motor(byte id,byte d1, byte d2, byte p1, byte e1,byte e2);
    Motor();
    void Forward();
    void Backward();
    void Stop();
    void defineOutput();
    void changePWM(byte p);
    int getTicks();
    void setTicks(int ticks);
    void _IPID(double actual,double target);
    double _OPID();
    
};


#endif
