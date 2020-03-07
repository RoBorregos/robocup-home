Movement::Movement() {
                    //id,d1,d2,p1,e1,e2
  this->B_left  = Motor(1,51,50,4,3,15);
  this->F_left  = Motor(2,37,36,5,2,22);
  this->B_right = Motor(3,53,52,6,19,14);
  this->F_right = Motor(4,25,24,7,18,23);
}

void Movement::pwm(int pwm) {
  this->B_right.changePWM(pwm);
  this->F_right.changePWM(pwm);
  this->B_left.changePWM(pwm);
  this->F_left.changePWM(pwm);
}

//PID
void  Movement::constantSpeed(){
  this->F_right.constantSpeed();
  this->F_left.constantSpeed();
  this->B_left.constantSpeed();
  this->B_right.constantSpeed();
}

double Movement::getVelocity(Motor A){
  double revolutions=A.ticks/PULSES_PER_REVOLUTION;
  return (revolutions*WHEEL_DIAMETER*PI_C)/(TIME_VELOCITY_SAMPLE/1000);
}
void Movement::calcVelocityaux(Motor &A){
  if(millis()-A.VelocityTiming < TIME_VELOCITY_SAMPLE)
    return;
  
  A.velocity=getVelocity(A);
  A.lastticks=A.ticks;  
  A.ticks=0;
  A.VelocityTiming=millis();

}
void Movement::calcVelocity(){
  calcVelocityaux(this->B_right);
  calcVelocityaux(this->F_right);
  calcVelocityaux(this->B_left);
  calcVelocityaux(this->F_left);
}

double Movement::getTargetTicks(){
  double ticks = getTargetVelocity();
  ticks=ticks * (TIME_VELOCITY_SAMPLE/1000);
  ticks=ticks/(WHEEL_DIAMETER*PI_C);  
  return ceil(ticks*PULSES_PER_REVOLUTION);
}

double Movement::getTargetVelocity(){
  return sqrt(this->dX*this->dX+this->dY*this->dY);
}
double Movement::getTargetAngle(){
  return atan(this->dY/this->dX);
}
double Movement::getOrientationA(){
  return this->angle;
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
//REFERENCE 0° right

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


