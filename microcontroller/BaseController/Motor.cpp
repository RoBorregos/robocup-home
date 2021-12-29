#include <math.h>
#include "Encoder.h"
#include "Motor.h"

//////////////////////////////////Constructor//////////////////////////////////////
Motor::Motor() {}
Motor::Motor(const MotorId id, const uint8_t digital_one, const uint8_t digital_two, 
const uint8_t analog_one, const uint8_t encoder_one, const uint8_t encoder_two) {

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
  
  current_state_ = MotorState::Forward;
}

void Motor::backward() {

  analogWrite(analog_one_, pwm_);
  
  if(current_state_ == MotorState::Backward) {
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);

  current_state_ = MotorState::Backward;
}

void Motor::stop() {
  analogWrite(analog_one_, LOW);

  if(current_state_ == MotorState::Stop) {
    return;
  }

  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);
  
  current_state_ = MotorState::Stop;
}

//////////////////////////////////Velocity//////////////////////////////////////
void Motor::setMotorSpeed(const double pwm) {
  pwm_ = abs(static_cast<int>(pwm));
  
  if(pwm < 0) {
    backward();
  } else if(pwm > 0) {
    forward();
  } else {
    stop();
  }
}

//////////////////////////////////Set Methods//////////////////////////////////////
void Motor::setEncoderTicks(const int encoder_ticks) {
  encoder_ticks_ = encoder_ticks;
}

//////////////////////////////////Get Methods//////////////////////////////////////
int Motor::getEncoderTicks() {
  return encoder_ticks_;
}

MotorState Motor::getCurrentState() {
  return current_state_;
}
