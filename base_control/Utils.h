#ifndef UtilClass

#define UtilClass

//UTILS
class Utils {
  public:
  
    Utils();
    
    int AngleToDirection(int angle);
    void sendToPC_velocity();
    void sendToPC_ticks();
    void sendToPC_ticksT();
    void sendToPC(double* a,double* b,double* c,double* d);
    
    unsigned long timeMessage;
    


};

#endif
