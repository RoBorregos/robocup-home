Motor::Motor() {}

Motor::Motor(byte id,byte d1, byte d2, byte p1, byte e1,byte e2) : _PID() {
  this->id = id;
  this->d1 = d1;
  this->d2 = d2;
  this->p1 = p1;
  this->e1 = e1;
  this->e2 = e2;
  this->Stop();
  this->defineOutput();
  this->changePWM(0);

  //PID
  _PID.setTunings(kp,ki,kd);
  _PID.setOutputLimits(0, 255);
  _PID.setMaxErrorSum(4000);
  _PID.setSampleTime(MOTOR_TIME_VELOCITY_SAMPLE);
}

double Motor::getTargetRPM(double velocity){
  return ((getTargetTicks(velocity)/PULSES_PER_REVOLUTION)*10)+velocityAdjustment;
}
double Motor::getTargetTicks(double velocity){
  double ticks =velocity * (MOTOR_TIME_VELOCITY_SAMPLE/1000);
  ticks=ticks/(WHEEL_DIAMETER*M_PI);  
  return ceil(ticks*PULSES_PER_REVOLUTION);
}

void Motor::constantSpeed(double velocity){
  _PID.Compute(getTargetRPM(velocity),speedActual,pwm,ticks,PULSES_PER_REVOLUTION);
  changePWM(pwm);
}

void Motor::defineOutput() {
  pinMode(this->d1, OUTPUT);
  pinMode(this->d2, OUTPUT);
  pinMode(this->p1, OUTPUT);
  pinMode(this->e1, INPUT);
  pinMode(this->e2, INPUT);
  
  switch (this->id){
    case 1:
      attachInterrupt(digitalPinToInterrupt(this->e1), Encoder::BLencoder, RISING);
    break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(this->e1), Encoder::FLencoder, RISING);
    break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(this->e1), Encoder::BRencoder, RISING);
    break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(this->e1), Encoder::FRencoder, RISING);
    break;
  }
}

int Motor::getTicks() {
  return this->ticks;
}
int Motor::getOdomTicks() {
  return this->odomTicks;
}

void Motor::setTicks(int ticks) {
  this->ticks=ticks;
}

void Motor::changePWM(double p){
  this->pwm=p;
  switch(actualState){
    case forward:
      this->Forward();
    break;
    case backward:
      this->Backward();
    break;
    case stop:
      this->Stop();
    break;
  }
}

void Motor::Stop() {
  analogWrite(this->p1, 0);
  digitalWrite(this->d1, 0);
  digitalWrite(this->d2, 0);
  
  if(this->actualState!=stop){
    _PID.reset();
  }

  this->actualState=stop;
}
void Motor::Forward() {
  analogWrite(this->p1, this->pwm);
  digitalWrite(this->d1, 1);
  digitalWrite(this->d2, 0);

  if(this->actualState!=forward){
    _PID.reset();
  }

  this->actualState=forward;
}
void Motor::Backward() {
  analogWrite(this->p1, this->pwm);
  digitalWrite(this->d1, 0);
  digitalWrite(this->d2, 1);
  
  if(this->actualState!=backward){
    _PID.reset();
  }
  
  this->actualState=backward;
}
