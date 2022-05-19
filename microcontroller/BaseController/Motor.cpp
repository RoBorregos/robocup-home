#include <math.h>
#include "Encoder.h"
#include "Motor.h"

//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const MotorId id, const uint8_t digital_one, const uint8_t digital_two, 
const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two) :
pid_(kP, kI, kD, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample) {

  id_ = id;
  digital_one_ = digital_one;
  digital_two_ = digital_two;
  analog_one_ = analog_one;
  encoder_one_ = encoder_one;
  encoder_two_ = encoder_two;

  defineOutput();
  stop();

}


//////////////////////////////////Initialization//////////////////////////////////////
void Motor::defineOutput() {
  pinMode(digital_one_, OUTPUT);
  pinMode(digital_two_, OUTPUT);
  pinMode(analog_one_, OUTPUT);
  pinMode(encoder_one_, INPUT_PULLUP);
  pinMode(encoder_two_, INPUT_PULLUP);
}

void Motor::initEncoders() {
  switch (id_) {
    case MotorId::BackLeft:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::backLeftEncoder, RISING);
    break;
    case MotorId::FrontLeft:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::frontLeftEncoder, RISING);
    break;
    case MotorId::BackRight:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::backRightEncoder, RISING);
    break;
    case MotorId::FrontRight:
      attachInterrupt(digitalPinToInterrupt(encoder_one_), Encoder::frontRightEncoder, RISING);
    break;
  }
}

//////////////////////////////////Motor State//////////////////////////////////////
void Motor::forward() {
  analogWrite(analog_one_, pwm_);

  if(current_state_ == MotorState::Forward) {
    return;
  }
  
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, LOW);

  pid_.reset();

  last_state_ =  (last_state_ != current_state_) ? current_state_ : last_state_;
  current_state_ = MotorState::Forward;
}

void Motor::backward() {
  analogWrite(analog_one_, pwm_);
  
  if(current_state_ == MotorState::Backward) {
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);

  pid_.reset();

  last_state_ =  (last_state_ != current_state_) ? current_state_ : last_state_;
  current_state_ = MotorState::Backward;
}

void Motor::stop() {
  analogWrite(analog_one_, LOW);

  if(current_state_ == MotorState::Stop) {
    return;
  }

  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);
  
  pid_.reset();

  last_state_ =  (last_state_ != current_state_) ? current_state_ : last_state_;
  current_state_ = MotorState::Stop;
}

//////////////////////////////////Velocity//////////////////////////////////////
double mapD(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double constrainD(double x, double min_, double max_) {
  if (x < min_) {
    return min_;
  }
  if (x > max_) {
    return max_;
  }
  return x;
}

double Motor::approxSpeedToPWM(double target_speed) {
  target_speed = fabs(target_speed);
  target_speed = constrainD(target_speed, 0, kRPM);
  double pwmValue = mapD(target_speed, 0, kRPM, 0, kMaxPWM);
  
  // Enforce deadzone around 0.
  if(pwmValue <= kPwmDeadZone) {
    pwmValue = 0;
  }
  
  // Enforce min pwm required for motor movement.
  if(pwmValue <= kMinPwmForMovement && pwmValue != 0) {
    pwmValue = kMinPwmForMovement;
  }
  
  return pwmValue;
}

void Motor::setMotorSpeed(const double target_speed) {
  int speed_sign = min(1, max(-1, target_speed * 1000));
  target_speed_ = fabs(target_speed);
  pwm_ = approxSpeedToPWM(target_speed);

  switch(speed_sign) {
    case 0:
      stop();
    break;
    case -1:
      backward();
    break;
    case 1:
      forward();
    break;
  }
}

void Motor::setMotorSpeedPID(const double target_speed) {
  int speed_sign = min(1, max(-1, target_speed * 1000));
  target_speed_ = fabs(target_speed);
  double tmp_pwm = pwm_;

  switch(speed_sign) {
    case 0:
      stop();
    break;
    case -1:
      backward();
    break;
    case 1:
      forward();
    break;
  }

  pid_.compute(
    getTargetRps(target_speed_), current_speed_, tmp_pwm, pid_ticks_,
    kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond
  );

  changePwm(tmp_pwm);
}

double Motor::getTargetRps(const double velocity) {
  return MsToRps(velocity);
}

double Motor::MsToRps(const double ms) {
  return  (ms / ( M_PI * kWheelDiameter));
}

double Motor::RpsToMs(const double rps) {
  return  (M_PI * kWheelDiameter * rps);
}

double Motor::RpmToMs(const double rpm) {
  return  (RpsToMs(rpm)/60);
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

//////////////////////////////////Set Methods//////////////////////////////////////
void Motor::setEncoderTicks(const long encoder_ticks) {
  encoder_ticks_ = encoder_ticks;
}
void Motor::deltaEncoderTicks(const int delta_encoder_ticks) {
  encoder_ticks_ = encoder_ticks_ + delta_encoder_ticks;
}
void Motor::setPidTicks(const int pid_ticks) {
  pid_ticks_ = pid_ticks;
}
void Motor::deltaPidTicks(const int delta_pid_ticks) {
  pid_ticks_ = pid_ticks_ + delta_pid_ticks;
}

//////////////////////////////////Get Methods//////////////////////////////////////
long Motor::getEncoderTicks() {
  return encoder_ticks_;
}
int Motor::getPidTicks() {
  return pid_ticks_;
}

double Motor::getCurrentSpeed() {
  return (RpsToMs(current_speed_));
  //return current_speed_;
}

double Motor::getTargetSpeed() {
  return target_speed_;
}

MotorState Motor::getCurrentState() {
  return current_state_;
}

MotorState Motor::getlastState() {
  return last_state_;
}

int Motor::getEnc1State() {
  return digitalRead(encoder_one_);
}

int Motor::getEnc2State() {
  return digitalRead(encoder_two_);
}