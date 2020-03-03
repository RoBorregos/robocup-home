Movement::Movement() {
  this->B_left  = Motor(1,46,47,9,19,15);
  this->F_left  = Motor(2,44,45,4,3,22);
  this->B_right = Motor(3,49,48,8,18,14);
  this->F_right = Motor(4,52,53,5,2,23);
}

void Movement::pwm(int pwm) {
  this->B_right.changePWM(pwm);
  this->F_right.changePWM(pwm);
  this->B_left.changePWM(pwm);
  this->F_left.changePWM(pwm);
}

void Movement::getDistance(Motor A){
  double revolutions=A.ticks/PULSES_PER_REVOLUTION;
  return (revolutions*WHEEL_DIAMETER*PI_C)/(TIME_VELOCITY_SAMPLE/1000);
}

void Movement::calcVelocity(){
  if(millis()-this->VelocityTiming<TIME_VELOCITY_SAMPLE)
    return;

  this->VelocityTiming=millis();
  this->B_right.velocity=getDistance(B_right);
  this->F_right.velocity=getDistance(F_right);
  this->B_left.velocity=getDistance(B_left);
  this->F_left.velocity=getDistance(F_left);
}

void Movement::setDirection(int angle){
  
  switch(angle){
    case 0:
      this->_move0();
    break;
    case 45:
      this->_move45();
    break;
    case 90:
      this->_move90();
    break;
    case 135:
      this->_move135();
    break;
    case 180:
      this->_move180();
    break;
    case 225:
      this->_move225();
    break;
    case 270:
      this->_move270();
    break;
    case 315:
      this->_move315();
    break;
  }
}

//DIRECTIONS 
//REFERENCE 0° Right

void Movement::_move0() {
  this->F_left.Forward();
  this->B_left.Backward();
  this->F_right.Backward();
  this->B_right.Forward();
}
void Movement::_move45() {
  this->F_left.Forward();
  this->B_left.Stop();
  this->F_right.Stop();
  this->B_right.Forward();
}
void Movement::_move90() {
  this->F_left.Forward();
  this->B_left.Forward();
  this->F_right.Forward();
  this->B_right.Forward();
}
void Movement::_move135() {
  this->F_left.Stop();
  this->B_left.Forward();
  this->F_right.Forward();
  this->B_right.Stop();
}
void Movement::_move180() {
  this->F_left.Backward();
  this->B_left.Forward();
  this->F_right.Forward();
  this->B_right.Backward();
}
void Movement::_move225() {
  this->F_left.Backward();
  this->B_left.Stop();
  this->F_right.Stop();
  this->B_right.Backward();
}
void Movement::_move270() {
  this->F_left.Backward();
  this->B_left.Backward();
  this->F_right.Backward();
  this->B_right.Backward();
}
void Movement::_move315() {
  this->F_left.Stop();
  this->B_left.Backward();
  this->F_right.Backward();
  this->B_right.Stop();
}
void Movement::_rotateL() {
  this->F_left.Backward();
  this->B_left.Backward();
  this->F_right.Forward();
  this->B_right.Forward();
}
void Movement::_rotateR() {
  this->F_left.Forward();
  this->B_left.Forward();
  this->F_right.Backward();
  this->B_right.Backward();
}
void Movement::_stop() {
  this->F_left.Stop();
  this->B_left.Stop();
  this->F_right.Stop();
  this->B_right.Stop();
}
