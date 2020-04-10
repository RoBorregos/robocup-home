#ifndef PID_h
#define PID_h

class PID{

  public:
  

    PID();
    PID(const double kp ,const double ki,const double kd);

    //Motor
    void Compute(double setpoint,double &input,double &output,int &resetV);
    
    //BNO
    void Compute(double setpoint,double input,double &output);
    void Compute(double error,double &output,byte flag);

    
    void setTunings(const double kp, const double ki, const double kd);         	  
    void setSampleTime(const unsigned long sampleTime);			
    void setMaxErrorSum(const double maxErrorSum);
    void setOutputLimits(const double outMin, const double outMax);
    void reset();
    
    double getKp();
    double getKi();
    double getKd();

    double getPre();
    

  private:
	
    double kp=0;
    double ki=0;
    double kd=0;

    double errorSum=0;
    double errorPre=0;
   
    double maxErrorSum;
	  double outMin, outMax;
    
    unsigned long sampleTime;
    unsigned long time;

};
#endif

