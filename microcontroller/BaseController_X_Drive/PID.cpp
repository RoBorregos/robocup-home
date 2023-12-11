
#include "PID.h"

//////////////////////////////////Constructor//////////////////////////////////////
PID::PID() {
  time_ = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time) {
  time_ = millis();
  setTunings(kp, ki, kd);
  setOutputLimits(out_min, out_max);
  setMaxErrorSum(max_error_sum);
  setSampleTime(sample_time);
}

//////////////////////////////////Compute//////////////////////////////////////
void PID::compute(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,const double count_time_samples_in_one_second) {
  if(millis()-time_ < sample_time_) {
      return;
  }
  // Improvement: Change Time to millis() instead of fixed variable.
  input = (reset_variable / pulses_per_rev) * count_time_samples_in_one_second;
  reset_variable = 0;
  
  const double error = setpoint - input;
  output = error * kp_ + error_sum_ * ki_ + (error - error_pre_) * kd_;
  
  error_pre_ = error;
  error_sum_ += error;
  

  error_sum_ = max(max_error_sum_ * -1, min(max_error_sum_, error_sum_));
  output = max(out_min_, min(out_max_, output));
  
  time_ = millis();
  
}

void PID::compute(const double setpoint, const double input, double &output) {
  if(millis()-time_ < sample_time_) {
      return;
  }

  const double error = setpoint - input;
  output = error * kp_ + error_sum_ * ki_ + (error - error_pre_) * kd_;
  
  error_pre_ = error;
  error_sum_ += error;
  

  error_sum_ = max(max_error_sum_ * -1, min(max_error_sum_, error_sum_));
  output = max(out_min_, min(out_max_, output));
  
  time_ = millis();
  
}

double PID::compute_dt(const double setpoint, const double input, const double sample_time_) {

  const double error = setpoint - input;
  
  double output = output1 + (kp_ + kd_ / sample_time_) * error +
  ((kp_) * (-1) + ki_ * sample_time_ - 2 * kd_ / sample_time_) * error1 + (kd_ / sample_time_) * error2;

  output1 = output;
  error2 = error1;
  error1 = error;
  
  if(output > 100.0)
    output = 100.0;
  if(output < 1.0)
    output = 1.0;
  

  //output = max(out_min_, min(out_max_, output));
  
  time_ = millis();
    
  return output;

}

//////////////////////////////////Set Methods//////////////////////////////////////
void PID::setTunings(const double kp,  const double ki , const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}     	 

void PID::setSampleTime(const unsigned long sample_time) {
  sample_time_ = sample_time;
}				

void PID::setMaxErrorSum(const double max_error_sum) {
  max_error_sum_ = max_error_sum;
}

void PID::setOutputLimits(const double out_min, const double out_max) {
  out_min_ = out_min;
  out_max_ = out_max;
}

void PID::reset() {
  error_sum_ = 0;
  error_pre_ = 0;
}

//////////////////////////////////Get Methods//////////////////////////////////////
double PID::getKp() {
  return kp_;
}

double PID::getKi() {
  return ki_;
}

double PID::getKd() {
  return kd_;
}

double PID::getPre() {
  return error_pre_;
}
