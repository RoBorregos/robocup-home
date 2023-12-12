// This class is a proportional–integral–derivative controller used to control 
// different systems of the robot. 
#ifndef PID_h
#define PID_h
#include <math.h>
#include <Arduino.h>

class PID{
  public:

    //////////////////////////////////Constructor//////////////////////////////////////
    PID();
    PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time);
    
    //////////////////////////////////Compute//////////////////////////////////////
    // Computes an output accoriding to the error and PID constants.
    void compute(const double error, double &output, const byte flag);
    
    // Computes an output accoriding to the error calculated internally and PID constants.
    void compute(const double setpoint, const double input, double &output);

    double compute_dt(const double setpoint, const double input, const double sample_time_);
    
    // Computes an output accoriding to the error calculated internally and PID constants,
    // also it resets a variable (used with ticks).
    void compute(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,const double count_time_samples_in_one_second);
    
    //////////////////////////////////Set Methods//////////////////////////////////////
    // This function set kp_, ki_, kd_ variables.
    void setTunings(const double kp,  const double ki,  const double kd);         	  
    
    // This function set sample_time_ variable.
    void setSampleTime(const unsigned long sample_time);			
    
    // This function set max_error_sum_ variable.
    void setMaxErrorSum(const double max_error_sum);
    
    // This function set out_min_ and out_max_ variables.
    void setOutputLimits(const double out_min, const double out_max);
    
    //This function resets the error_sum_ and error_pre_.
    void reset();
    
    //////////////////////////////////Get Methods//////////////////////////////////////
    // Return kp_ variable.
    double getKp();
    
    // Return ki_ variable.
    double getKi();
    
    // Return kd_ variable.
    double getKd();
    
    // Return las error variable.
    double getPre();
  
  private:

    double kp_ = 0;
    double ki_ = 0;
    double kd_ = 0;

    double output1 = 0;
    double error1 = 0;
    double error2 = 0;
    double error_sum_ = 0;
    double error_pre_ = 0;
   
    double max_error_sum_;
	  double out_min_, out_max_;
    
    unsigned long sample_time_;
    unsigned long time_;

};
#endif

