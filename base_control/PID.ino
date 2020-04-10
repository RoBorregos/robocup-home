
PID::PID(){
  time=millis();
}

PID::PID(const double kp, const double ki, const double kd){
  time=millis();
  this->setTunings(kp,ki,kd);
}


void PID::Compute(double setpoint,double &input,double &output,int &resetV){
  if(millis()-time < sampleTime)
      return;

  resetV=0;
  
  double error = setpoint - input;
  output = error*kp + errorSum*ki + (error - errorPre)*kd;
  
  errorPre = error;
  errorSum += error;
  

  errorSum=max(maxErrorSum*-1,min(maxErrorSum,errorSum));
  output=max(outMin,min(outMax,output));
  
  time=millis();
  
}

void PID::Compute(double setpoint,double input,double &output){
  if(millis()-time < sampleTime)
      return;


  double error = setpoint - input;
  output = error*kp + errorSum*ki + (error - errorPre)*kd;
  
  errorPre = error;
  errorSum += error;
  

  errorSum=max(maxErrorSum*-1,min(maxErrorSum,errorSum));
  output=max(outMin,min(outMax,output));
  
  time=millis();
  
}
void PID::Compute(double error,double &output,byte flag){
  if(millis()-time < sampleTime)
      return;

  

  if(errorPre*error<=0){
    errorPre=0;
    errorSum=0;
  }
  if(flag==0)
  if(abs(error)<=2){
    errorPre=0;
    errorSum=0;
  }
  
  output = error*kp + errorSum*ki + (error - errorPre)*kd;
  errorPre = error;
  errorSum += error;
  
  
  errorSum=max(maxErrorSum*-1,min(maxErrorSum,errorSum));
  output=max(outMin,min(outMax,output));
  
  time=millis();

}
void PID::setTunings(const double kp, const double ki ,const double kd){
  this->kp=kp;
  this->ki=ki;
  this->kd=kd;
}     	  
void PID::setSampleTime(const unsigned long sampleTime){
  this->sampleTime=sampleTime;
}					  
void PID::setMaxErrorSum(const double maxErrorSum){
  this->maxErrorSum=maxErrorSum;
}					  
void PID::setOutputLimits(const double outMin,const double outMax){
  this->outMin=outMin;
  this->outMax=outMax;
}
void PID::reset(){
  errorSum = 0;
  errorPre = 0;
}
double PID::getKp(){
  return kp;
}
double PID::getKi(){
  return ki;
}
double PID::getKd(){
  return kd;
}
double PID::getPre(){
  return errorPre;
}

