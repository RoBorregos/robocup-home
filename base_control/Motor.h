#ifndef MotorClass

#define MotorClass
class Motor {
  public:
    //PWM value
    int pwm = 0;
    //PWM PINS
    byte m1 = 0;
    byte m2 = 0;
    //STATE
    int state = 0;

    //
    Motor(byte a, byte b);
    Motor();
    void Forward();
    void Backward();
    void Stop();
    void defineOutput();
    void changePWM(byte p);
};
Motor

#endif
