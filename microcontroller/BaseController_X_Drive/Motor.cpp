#include <math.h>
#include "Encoder.h"
#include "Motor.h"

//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const MotorId id, const uint8_t digital_one, const uint8_t digital_two, 
const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two) : 
pid_(kP, kI, kD, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidSampleTime) {
  id_ = id;
  digital_one_ = digital_one;
  digital_two_ = digital_two;
  analog_one_ = analog_one;
  encoder_one_ = encoder_one;
  encoder_two_ = encoder_two;

  stop();
  defineOutput();

}


//////////////////////////////////Initialization//////////////////////////////////////
void Motor::defineOutput() {
  pinMode(digital_one_, OUTPUT);
  pinMode(digital_two_, OUTPUT);
  pinMode(analog_one_, OUTPUT);
  pinMode(encoder_one_, INPUT);
  pinMode(encoder_two_, INPUT);
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
  
  current_state_ = MotorState::Stop;
}


//////////////////////////////////Velocity//////////////////////////////////////
void  Motor::RpmToPwm(const double rpm){
  pwm_ = rpm * (255.0 / kMaxRpm);
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

void Motor::stableRPM(const double velocity) {
  currentMillis = millis();
  target_speed_ = velocity;
  if((currentMillis - prevMillis) >= kPidSampleTime){
    prevMillis = currentMillis;
    current_speed_ = 10 * pid_ticks_ * (60 / kPulsesPerRevolution);
    setPidTicks(0);
  }
  
  //Serial.println(getCurrentSpeed());

  int speed_sign = fmin(1, fmax(-1, velocity));
  //velocity = fabs(velocity);
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
  
  //Need to change velocity into RPM units
  tmp_pwm = pid_.compute_dt(abs(target_speed_), getCurrentSpeed(), kPidMotorSampleTime);
  //Serial.print("Setpoint rpm: "); Serial.println(velocity);
  Serial.print("Current Motor rpm: "); Serial.println(getCurrentSpeed());
  RpmToPwm(tmp_pwm);
  //Serial.print("PWM value: "); Serial.println(pwm_);

  changePwm(pwm_);
}

//////////////////////////////////Set Methods//////////////////////////////////////
void Motor::setPidTicks(const int pid_ticks) {
  pid_ticks_ = pid_ticks;
}

void Motor::setOdomTicks(const int odom_ticks) {
  odom_ticks_ = odom_ticks;
}

void Motor::setEncodersDir(const int encoders_dir){
  encoders_dir_ = encoders_dir;
}


//////////////////////////////////Get Methods//////////////////////////////////////
int Motor::getEncodersDir(){
  return encoders_dir_;
}

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
