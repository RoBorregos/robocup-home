//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const uint8_t id,const uint8_t digital_one,const uint8_t digital_two,const uint8_t analog_one,const uint8_t encoder_one,const uint8_t encoder_two) : pid_() {
  id_ = id;
  digital_one_ = digital_one;
  digital_two_ = digital_two;
  analog_one_ = analog_one;
  encoder_one_ = encoder_one;
  encoder_two_ = encoder_two;

  stop();
  defineOutput();
  changePwm(LOW);

  //PID
  pid_.setTunings(kP,kI,kD);
  pid_.setOutputLimits(kPidMinOutputLimit,kPidMaxOutputLimit);
  pid_.setMaxErrorSum(kPidMaxErrorSum);
  pid_.setSampleTime(kPidMotorTimeSample);
}


//////////////////////////////////Initialization//////////////////////////////////////
void Motor::defineOutput() {
  pinMode(digital_one_, OUTPUT);
  pinMode(digital_two_, OUTPUT);
  pinMode(analog_one_, OUTPUT);
  pinMode(encoder_one_, INPUT);
  pinMode(encoder_two_, INPUT);
}
void Motor::initEncoders(){
  switch (id_){
    case kIdBackLeftMotor:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::backLeftEncoder, RISING);
    break;
    case kIdFrontLeftMotor:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::frontLeftEncoder, RISING);
    break;
    case kIdBackRightMotor:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::backRightEncoder, RISING);
    break;
    case kIdFrontRightMotor:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::frontRightEncoder, RISING);
    break;
  }
}

//////////////////////////////////Motor State//////////////////////////////////////
void Motor::forward() {
  analogWrite(analog_one_,pwm_);
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, LOW);

  if(actual_state_ != Forward){
    pid_.reset();
  }

  actual_state_ = Forward;
}
void Motor::backward() {
  analogWrite(analog_one_,pwm_);
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);
  
  if(actual_state_ != Backward){
    pid_.reset();
  }
  
  actual_state_ = Backward;
}
void Motor::stop() {
  analogWrite(analog_one_, LOW);
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);
  
  if(actual_state_ != Stop){
    pid_.reset();
  }

  actual_state_ = Stop;
}

//////////////////////////////////Velocity//////////////////////////////////////
double Motor::getTargetRpm(const double velocity){
  return ((getTargetTicks(velocity)/kPulsesPerRevolution)*kPidCountTimeSamplesInOneSecond)+velocity_adjustment_;
}
double Motor::getTargetTicks(const double velocity){
  double ticks = velocity * (kPidMotorTimeSample/kOneSecondInMillis);
  ticks = ticks/(kWheelDiameter*M_PI);  
  return ceil(ticks*kPulsesPerRevolution);
}
void Motor::changePwm(const uint8_t pwm){
  pwm_ = pwm;
  switch(actual_state_){
    case Forward:
      forward();
    break;
    case Backward:
      backward();
    break;
    case Stop:
      stop();
    break;
  }
}
void Motor::constantSpeed(const double velocity){
  double tmp_pwm = pwm_;
  pid_.compute(getTargetRpm(velocity),current_speed_,tmp_pwm,pid_ticks_,kPulsesPerRevolution);
  changePwm(tmp_pwm);
}

//////////////////////////////////Set Methods//////////////////////////////////////
void Motor::setPidTicks(const int pid_ticks) {
  pid_ticks_ = pid_ticks;
}
void Motor::setOdomTicks(const int odom_ticks) {
  odom_ticks_ = odom_ticks;
}
void Motor::setVelocityAdjustment(const double velocity_adjustment){
  velocity_adjustment_ = velocity_adjustment;
}

//////////////////////////////////Get Methods//////////////////////////////////////
int Motor::getPidTicks() {
  return pid_ticks_;
}
int Motor::getOdomTicks() {
  return odom_ticks_;
}
double Motor::getLastTicks(){
  return last_ticks_;
}
double Motor::getCurrentSpeed(){
  return current_speed_;
}
motorState Motor::getActualState(){
  return actual_state_;
}