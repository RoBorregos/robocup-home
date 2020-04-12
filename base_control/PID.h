//This class is a proportional–integral–derivative controller used to control different systems of the robot. 
#ifndef PID_h
#define PID_h

class PID{

  public:
  
    //////////////////////////////////Constructor//////////////////////////////////////
    PID();
    PID(const double kp ,const double ki,const double kd);

    //////////////////////////////////Compute//////////////////////////////////////
    ///This function computes an output accoriding to the error and PID constants.
    void compute(const double error,double &output,const byte flag);
    ///This function computes an output accoriding to the error calculated internally and PID constants.
    void compute(const double setpoint,const double input,double &output);
    ///This function computes an output accoriding to the error calculated internally and PID constants, also it resets a variable (used with ticks).
    void compute(const double setpoint,double &input,double &output,int &reset_variable,const int pulses_per_rev);
    
    //////////////////////////////////Set Methods//////////////////////////////////////
    void setTunings(const double kp, const double ki, const double kd);         	  
    void setSampleTime(const unsigned long sample_time);			
    void setMaxErrorSum(const double max_error_sum);
    void setOutputLimits(const double out_min, const double out_max);
    void reset();
    
    //////////////////////////////////Get Methods//////////////////////////////////////
    double getKp();
    double getKi();
    double getKd();
    double getPre();
  
  private:
    
    double kp_=0;
    double ki_=0;
    double kd_=0;

    double error_sum_=0;
    double error_pre_=0;
   
    double max_error_sum_;
	  double out_min_, out_max_;
    
    unsigned long sample_time_;
    unsigned long time_;

};
#endif

