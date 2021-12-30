#include "Encoder.h"
//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor) {
  motor.deltaPidTicks(1);
  if(motor.getCurrentState() == MotorState::Forward) {
    motor.deltaEncoderTicks(1);
  }
  else {
    motor.deltaEncoderTicks(-1);
  }
}

//////////////////////////////////Motor Functions//////////////////////////////////////
// Robot* is declared in BaseController.ino. It is used as a global variable to use it in
// the static functions that are required by the attachInterrupts of the encoders.
void Encoder::backLeftEncoder() {
  handleEncoder(robot->back_left_motor_);
}

void Encoder::backRightEncoder() {
  handleEncoder(robot->back_right_motor_);
}

void Encoder::frontLeftEncoder() {
  handleEncoder(robot->front_left_motor_);
}

void Encoder::frontRightEncoder() {
  handleEncoder(robot->front_right_motor_);
}
