#include <math.h>
#include "Encoder.h"
#include "Motor.h"

//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const MotorId id, const uint8_t fwd_rev, const uint8_t speed, 
const uint8_t enable, const uint8_t encoder_one, const uint8_t encoder_two, bool inv_fwd) :
pid_(kP, kI, kD, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample) {

  id_ = id;
  fwd_rev_ = fwd_rev;
  speed_ = speed;
  enable_ = enable;
  encoder_one_ = encoder_one;
  encoder_two_ = encoder_two;
  inv_fwd_ = inv_fwd;

  stop();
  defineOutput();

}


//////////////////////////////////Initialization//////////////////////////////////////
void Motor::defineOutput() {
  pinMode(fwd_rev_, OUTPUT);
  pinMode(speed_, OUTPUT);
  pinMode(enable_, OUTPUT);
  pinMode(encoder_one_, INPUT);
  pinMode(encoder_two_, INPUT);
}

void Motor::initEncoders() {
  switch (id_) {
    case MotorId::Left:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::leftEncoder, RISING);
    break;
    case MotorId::Right:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::rightEncoder, RISING);
    break;
  }
}

//////////////////////////////////Motor State//////////////////////////////////////
void Motor::forward() {
  
  analogWrite(speed_, pwm_);

  if(current_state_ == MotorState::Forward) {
    return;
  }
  
  digitalWrite(fwd_rev_, inv_fwd_? LOW : HIGH);
  digitalWrite(enable_, LOW);

  pid_.reset();
  
  current_state_ = MotorState::Forward;
}

void Motor::backward() {

  analogWrite(speed_, pwm_);
  
  if(current_state_ == MotorState::Backward) {
    return;
  }
  
  digitalWrite(fwd_rev_, inv_fwd_? HIGH : LOW);
  digitalWrite(enable_, LOW);
  
  pid_.reset();

  current_state_ = MotorState::Backward;
}

void Motor::stop() {
  analogWrite(speed_, pwm_);

  if(current_state_ == MotorState::Stop) {
    return;
  }

  digitalWrite(fwd_rev_, LOW);
  digitalWrite(enable_, HIGH);
  
  pid_.reset();
  
  current_state_ = MotorState::Stop;
}

void Motor::setEncodersDir(const int encoders_dir){
  encoders_dir_ = encoders_dir;
}

int Motor::getEncodersDir(){
  return encoders_dir_;
}

//////////////////////////////////Velocity//////////////////////////////////////
double Motor::getTargetRps(const double velocity) {
  return MsToRps(velocity);
}

double Motor::getTargetTicks(const double velocity) {
  return RpsToTicks( MsToRps(velocity) );
}

double Motor::RpsToTicks(const double rps) {
  return  (rps / kPidCountTimeSamplesInOneSecond) * kPulsesPerRevolution;
}

double Motor::TicksToRps(const double ticks) {
  return  (ticks * kPidCountTimeSamplesInOneSecond) / kPulsesPerRevolution;
}

double Motor::RpsToMs(const double rps) {
  return  rps * M_PI * kWheelDiameter;
}

double Motor::MsToRps(const double ms) {
  return  (ms / ( M_PI * kWheelDiameter));
}


uint8_t Motor::getPWM(){
  return pwm_;
}

void Motor::changePwm(const uint8_t pwm) {
  pwm_ = pwm;
  switch(current_state_) {
    case MotorState::Forward:
      forward();
    break;
    case MotorState::Backward:
      backward();
    break;
    case MotorState::Stop:
      stop();
    break;
  }
}

void Motor::constantRPM(double velocity) {
  target_speed_ = RpsToMs(velocity / kSecondsInMinute);
  int speed_sign = fmin(1, fmax(-1, velocity));
  velocity = fabs(velocity);
  double tmp_pwm = pwm_;
  switch (speed_sign)
  {
    case 0:
      stop();
      break;
    case 1:
      forward();
      break;
    case -1:
      backward();
      break;
  }
  pid_.compute(
    velocity / kSecondsInMinute, current_speed_, tmp_pwm, pid_ticks_,
    kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond
  );
 
  changePwm(tmp_pwm);
}

//////////////////////////////////Set Methods//////////////////////////////////////
void Motor::setPidTicks(const int pid_ticks) {
  pid_ticks_ = pid_ticks;
}

void Motor::setOdomTicks(const int odom_ticks) {
  odom_ticks_ = odom_ticks;
}

//////////////////////////////////Get Methods//////////////////////////////////////
int Motor::getPidTicks() {
  return pid_ticks_;
}

int Motor::getOdomTicks() {
  return odom_ticks_;
}

double Motor::getLastTicks() {
  return last_ticks_;
}

double Motor::getTargetSpeed() {
  return target_speed_;
}

double Motor::getCurrentSpeed() {
  return current_speed_;
}

MotorState Motor::getCurrentState() {
  return current_state_;
}

uint8_t Motor::getEncoderOne() {
  return encoder_one_;
}

uint8_t Motor::getEncoderTwo() {
  return encoder_two_;
}
