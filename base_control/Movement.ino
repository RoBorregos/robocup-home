//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno) {
  bno_=bno;
  
  back_left_  = Motor(kIdBackLeftMotor,kDigitalPinsBackLeftMotor[0],kDigitalPinsBackLeftMotor[1],kAnalogPinBackLeftMotor,kEncoderPinsBackLeftMotor[0],kEncoderPinsBackLeftMotor[1]);
  front_left_ = Motor(kIdFrontLeftMotor,kDigitalPinsFrontLeftMotor[0],kDigitalPinsFrontLeftMotor[1],kAnalogPinFrontLeftMotor,kEncoderPinsFrontLeftMotor[0],kEncoderPinsFrontLeftMotor[1]);
  back_right_ = Motor(kIdBackRightMotor,kDigitalPinsBackRightMotor[0],kDigitalPinsBackRightMotor[1],kAnalogPinBackRightMotor,kEncoderPinsBackRightMotor[0],kEncoderPinsBackRightMotor[1]);
  front_right_= Motor(kIdFrontRightMotor,kDigitalPinsFrontRightMotor[0],kDigitalPinsFrontRightMotor[1],kAnalogPinFrontRightMotor,kEncoderPinsFrontRightMotor[0],kEncoderPinsFrontRightMotor[1]);

  //PID
  pid_straight_.setTunings(kPPidStraight,kIPidStraight,kDPidStraight);
  pid_straight_.setOutputLimits(kOutputMinLimitPidStraight,kOutputMaxLimitPidStraight);
  pid_straight_.setMaxErrorSum(kPidMaxErrorSum);
  pid_straight_.setSampleTime(kPidMovementTimeSample);

  pid_rotation_.setTunings(kPPidRotation,kIPidRotation,kDPidRotation);
  pid_rotation_.setOutputLimits(kOutputMinLimitPidRotation,kOutputMaxLimitPidRotation);
  pid_rotation_.setMaxErrorSum(kPidMaxErrorSum);
  pid_rotation_.setSampleTime(kPidMovementTimeSample);
}

//////////////////////////////////Encoders//////////////////////////////////////
void Movement::initEncoders(){
  back_left_.initEncoders();
  front_left_.initEncoders();
  back_right_.initEncoders();
  front_right_.initEncoders();
}

//////////////////////////////////PWM//////////////////////////////////////
void Movement::changePwm(int pwm) {
  back_left_.changePwm(pwm);
  front_left_.changePwm(pwm);
  back_right_.changePwm(pwm);
  front_right_.changePwm(pwm);
}


//////////////////////////////////VELOCITY//////////////////////////////////////
void Movement::setDeltaX(double delta_x){
  delta_x_=delta_x;
}
void Movement::setDeltaY(double delta_y){
  delta_y_=delta_y;
}
void Movement::setDeltaAngular(double delta_angular){
  delta_angular_=delta_angular;
}
double Movement::getTargetAngle(){
  return atan(delta_y_/delta_x_);
}
double Movement::getTargetLinearVelocity(){
  return sqrt(delta_x_*delta_x_+delta_y_*delta_y_);
}
double Movement::getTargetAngularVelocity(){
  return delta_angular_;
}
void Movement::stop(){
  front_left_.stop();
  back_left_.stop();
  front_right_.stop();
  back_right_.stop();
}

//////////////////////////////////DIRECTIONS//////////////////////////////////////
direction Movement::whereToGo(double &actual_angle){
    return whereToGo(actual_angle,target_angle_);
}
direction Movement::whereToGo(double &actual_angle,double target_angle){
    double actual_a = bno_->getActualAngle();
    actual_angle = actual_a;
    double diff_angle = int(abs(actual_a - target_angle)) % kMaxAngle; 
    actual_angle = diff_angle > kIntermediateAngle ? kMaxAngle - diff_angle : diff_angle;

    int sign = (actual_a - target_angle >= kMinAngle && actual_a - target_angle <= kIntermediateAngle) || (actual_a - target_angle <= kIntermediateAngle*-1 && actual_a - target_angle >= kMaxAngle*-1) ? 1 : -1; 
    actual_angle*=sign;

    if(sign!=1){
        return left;
    }
    return right;
}
int Movement::angleToDirection(int angle){
    int diff=kIntMax;
    for(int i=0;i<=kCountDirections;i++){
        if(diff > abs(angle-i*kDirectionSeparationAngle) ){
            diff=abs(angle-i*kDirectionSeparationAngle);
        }else{
            return (i-1)*kDirectionSeparationAngle;
        }
    }
    return 0;
}
void Movement::setDirection(int angle){
  switch(angle){
    case 0:
      move0();
    break;
    case 45:
      move45();
    break;
    case 90:
      move90();
    break;
    case 135:
      move135();
    break;
    case 180:
      move180();
    break;
    case 225:
      move225();
    break;
    case 270:
      move270();
    break;
    case 315:
      move315();
    break;
  }
}
void Movement::move0() {
  front_left_.forward();
  back_left_.backward();
  front_right_.backward();
  back_right_.forward();
}
void Movement::move45() {
  front_left_.forward();
  back_left_.stop();
  front_right_.stop();
  back_right_.forward();
}
void Movement::move90() {
  front_left_.forward();
  back_left_.forward();
  front_right_.forward();
  back_right_.forward();
}
void Movement::move135() {
  front_left_.stop();
  back_left_.forward();
  front_right_.forward();
  back_right_.stop();
}
void Movement::move180() {
  front_left_.backward();
  back_left_.forward();
  front_right_.forward();
  back_right_.backward();
}
void Movement::move225() {
  front_left_.backward();
  back_left_.stop();
  front_right_.stop();
  back_right_.backward();
}
void Movement::move270() {
  front_left_.backward();
  back_left_.backward();
  front_right_.backward();
  back_right_.backward();
}
void Movement::move315() {
  front_left_.stop();
  back_left_.backward();
  front_right_.backward();
  back_right_.stop();
}
void Movement::rotateLeft() {
  front_left_.backward();
  back_left_.backward();
  front_right_.forward();
  back_right_.forward();
  
}
void Movement::rotateRight() {
  front_left_.forward();
  back_left_.forward();
  front_right_.backward();
  back_right_.backward();
}

//////////////////////////////////PID//////////////////////////////////////
void Movement::constantLinearSpeed(){
  front_right_.constantSpeed(getTargetLinearVelocity());
  front_left_.constantSpeed(getTargetLinearVelocity());
  back_left_.constantSpeed(getTargetLinearVelocity());
  back_right_.constantSpeed(getTargetLinearVelocity());
}
void Movement::constantAngularSpeed(){
  front_right_.constantSpeed(getTargetAngularVelocity());
  front_left_.constantSpeed(getTargetAngularVelocity());
  back_left_.constantSpeed(getTargetAngularVelocity());
  back_right_.constantSpeed(getTargetAngularVelocity());
}
void Movement::velocityAdjustment(int adjustment){
  back_left_.setVelocityAdjustment((back_left_.getActualState() ==Forward )?adjustment*-1:adjustment);
  front_left_.setVelocityAdjustment((front_left_.getActualState() ==Forward )?adjustment*-1:adjustment);
  back_right_.setVelocityAdjustment((back_right_.getActualState()==Backward)?adjustment*-1:adjustment);
  front_right_.setVelocityAdjustment((front_right_.getActualState()==Backward)?adjustment*-1:adjustment);
}
void Movement::pidLinearMovement(){
    int angle=angleToDirection(getTargetAngle());
    setDirection(angle);
    constantLinearSpeed();

    double angle_error = 0;
    direction where = whereToGo(angle_error);
    pid_straight_.compute(angle_error, straight_output_,0);
    velocityAdjustment(straight_output_);
}
void Movement::pidAngularMovement(){
    if(delta_angular_<0){
        rotateRight();
    }else{
        rotateLeft();
    }
    constantAngularSpeed();
}
bool Movement::pidRotate(double target_angle){
    double output = 0;
    double angle_error = 0;
    direction where = whereToGo(angle_error,target_angle);
    if(abs(angle_error)<=kPidRotationTolerance && abs(pid_rotation_.getPre())<=kPidRotationTolerance && angle_error*pid_rotation_.getPre()>=0){
        stop();
        return true;
    }

    pid_rotation_.compute(angle_error, output,1);
    output = abs(output)+kOutputAdjustment;
    
    if(where==left){
        rotateRight();
    }else{
        rotateLeft();
    }

    back_left_.setVelocityAdjustment(output);
    front_left_.setVelocityAdjustment(output);
    back_right_.setVelocityAdjustment(output);
    front_right_.setVelocityAdjustment(output);
    constantAngularSpeed();
   
    return false;
}