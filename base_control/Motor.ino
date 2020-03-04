Motor::Motor() {}


Motor::Motor(byte id,byte d1, byte d2, byte p1, byte e1,byte e2) : _PID(0,0,0, Kp, Ki, Kd, DIRECT) {
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
  _PID.SetParams(0,0,0);
  _PID.SetOutputLimits(0, 255-MINPWM);
  _PID.SetTunings(Kp, Ki, Kd);
  _PID.SetSampleTime(1);
  _PID.SetMode(AUTOMATIC);
}

void Motor::_IPID(double actual,double target){
  _PID.SetInputs(actual, target);
}

double Motor::_OPID(){
  return _PID.GetOutput();
}

void Motor::defineOutput() {
  pinMode(this->d1, OUTPUT);
  pinMode(this->d2, OUTPUT);
  pinMode(this->p1, OUTPUT);
  pinMode(this->e1, INPUT);
  pinMode(this->e2, INPUT);
  
  switch (this->id){
    case 1:
      attachInterrupt(digitalPinToInterrupt(this->e1), BLencoder, RISING);
    break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(this->e1), FLencoder, RISING);
    break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(this->e1), BRencoder, RISING);
    break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(this->e1), FRencoder, RISING);
    break;
  }
}
int Motor::getTicks() {
  return this->ticks;
}
void Motor::setTicks(int ticks) {
  this->ticks=ticks;
}

void Motor::changePWM(byte p){
  this->pwm=p;
  switch(state){
    case 1:
      this->Forward();
    break;
    case 2:
      this->Backward();
    break;
    case 3:
      this->Stop();
    break;
  }
}

void Motor::Stop() {
  analogWrite(this->p1, 0);
  digitalWrite(this->d1, 0);
  digitalWrite(this->d2, 0);
  
  if(this->state!=3){
    this->lastticks=this->ticks;  
    this->ticks=0;
    this->VelocityTiming=millis();
  }

  this->state=3;
}
void Motor::Forward() {
  analogWrite(this->p1, this->pwm);
  digitalWrite(this->d1, 1);
  digitalWrite(this->d2, 0);

  if(this->state!=1){
    this->lastticks=this->ticks;  
    this->ticks=0;
    this->VelocityTiming=millis();
  }

  this->state=1;
}
void Motor::Backward() {
  analogWrite(this->p1, this->pwm);
  digitalWrite(this->d1, 0);
  digitalWrite(this->d2, 1);
  
  if(this->state!=2){
    this->lastticks=this->ticks;  
    this->ticks=0;
    this->VelocityTiming=millis();
  }
  
  this->state=2;
}
