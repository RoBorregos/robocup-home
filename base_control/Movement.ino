Movement::Movement(BNO *bno) {
  bno_=bno;
                    //id,d1,d2,p1,e1,e2
  this->B_left  = Motor(1,15,14,11,2,37);
  this->F_left  = Motor(2,37,36,5,2,22);
  this->B_right = Motor(3,45,43,6,19,14);
  this->F_right = Motor(4,25,24,7,18,23);

  //PID
  _PIDStraight.setTunings(0.0008, 0.00015, 0.0002);
  _PIDStraight.setOutputLimits(-0.175, 0.175);
  _PIDStraight.setMaxErrorSum(3000);
  _PIDStraight.setSampleTime(MOVEMENT_TIME_VELOCITY_SAMPLE);

  _PIDRotation.setTunings(0.00050, 0.00110, 0.00080);
  _PIDRotation.setOutputLimits(-0.18, 0.18);
  _PIDRotation.setMaxErrorSum(3000);
  _PIDRotation.setSampleTime(MOVEMENT_TIME_VELOCITY_SAMPLE);
}

//PWM
void Movement::pwm(int pwm) {
  this->B_right.changePWM(pwm);
  this->F_right.changePWM(pwm);
  this->B_left.changePWM(pwm);
  this->F_left.changePWM(pwm);
}

//VELOCITY
double Movement::getTargetLinearVelocity(){
  return sqrt(this->dX*this->dX+this->dY*this->dY);
}
double Movement::getTargetAngularVelocity(){
  return this->dteta;
}
double Movement::getTargetAngle(){
  return atan(this->dY/this->dX);
}

//DIRECTIONS
direction Movement::whereToGo(double &actualAngle){
    double actualA = bno_->actualAngle();
    actualAngle = actualA;
    double diffAngle = int(abs(actualA - targetAngle)) % 360; 
    actualAngle = diffAngle > 180 ? 360 - diffAngle : diffAngle;

    int sign = (actualA - targetAngle >= 0 && actualA - targetAngle <= 180) || (actualA - targetAngle <=-180 && actualA - targetAngle >= -360) ? 1 : -1; 
    actualAngle*=sign;

    if(sign!=1){
        return left;
    }
    return right;
}
direction Movement::whereToGo(double &actualAngle,double _targetAngle){
    double actualA = bno_->actualAngle();
    actualAngle = actualA;
    double diffAngle = int(abs(actualA - _targetAngle)) % 360; 
    actualAngle = diffAngle > 180 ? 360 - diffAngle : diffAngle;

    int sign = (actualA - _targetAngle >= 0 && actualA - _targetAngle <= 180) || (actualA - _targetAngle <=-180 && actualA - _targetAngle >= -360) ? 1 : -1; 
    actualAngle*=sign;

    if(sign!=1){
        return left;
    }
    return right;
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

//ROTATE
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
//STOP
void Movement::_stop() {
  this->F_left.Stop();
  this->B_left.Stop();
  this->F_right.Stop();
  this->B_right.Stop();
}

//PID
void Movement::constantLinearSpeed(){
  this->F_right.constantSpeed(getTargetLinearVelocity());
  this->F_left.constantSpeed(getTargetLinearVelocity());
  this->B_left.constantSpeed(getTargetLinearVelocity());
  this->B_right.constantSpeed(getTargetLinearVelocity());
}
void Movement::constantAngularSpeed(){
  this->F_right.constantSpeed(getTargetAngularVelocity());
  this->F_left.constantSpeed(getTargetAngularVelocity());
  this->B_left.constantSpeed(getTargetAngularVelocity());
  this->B_right.constantSpeed(getTargetAngularVelocity());
}
void Movement::doVelocityAdjustment(int adjustment){
  this->B_left.velocityAdjustment =(this->B_left.actualState ==forward )?adjustment*-1:adjustment;
  this->F_left.velocityAdjustment =(this->F_left.actualState ==forward )?adjustment*-1:adjustment;
  this->B_right.velocityAdjustment=(this->B_right.actualState==backward)?adjustment*-1:adjustment;
  this->F_right.velocityAdjustment=(this->F_right.actualState==backward)?adjustment*-1:adjustment;
}
void Movement::pidLinearMovement(){
    int angle=AngleToDirection(this->getTargetAngle());
    this->setDirection(angle);
    this->constantLinearSpeed();

    double angleError = 0;
    direction whereA = whereToGo(angleError);
    _PIDStraight.Compute(angleError, straightOutput,0);
    this->doVelocityAdjustment(straightOutput);
}
void Movement::pidAngularMovement(){
    if(this->dteta<0){
        this->_rotateR();
    }else{
        this->_rotateL();
    }
    this->constantAngularSpeed();
}
bool Movement::pidRotate(double _targetAngle){
    double output = 0;
    double angleError = 0;
    direction whereA = whereToGo(angleError,_targetAngle);
    Serial.println(angleError);
    if(abs(angleError)<=1 && abs(_PIDRotation.getPre())<=1 && angleError*_PIDRotation.getPre()>=0){
        
        this->_stop();
        return true;
    }

    _PIDRotation.Compute(angleError, output,1);
    output = abs(output)-0.075;
    
    if(whereA==left){
        this->_rotateR();
    }else{
        this->_rotateL();
    }

    this->B_left.velocityAdjustment=output;
    this->F_left.velocityAdjustment=output;
    this->B_right.velocityAdjustment=output;
    this->F_right.velocityAdjustment=output;
    this->constantAngularSpeed();
   
    return false;
}

//Utils
int Movement::AngleToDirection(int angle){
    int diff=1000;
    for(int i=0;i<=8;i++){
        if(diff > abs(angle-i*45) ){
            diff=abs(angle-i*45);
        }else{
            return (i-1)*45;
        }
    }
    return 0;
}
