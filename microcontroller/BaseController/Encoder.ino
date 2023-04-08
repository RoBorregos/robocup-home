#include "Encoder.h"
//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor) {
  motor.setPidTicks(motor.getPidTicks() + 1);
  if(motor.getCurrentState() == MotorState::Forward) {
    motor.setOdomTicks(motor.getOdomTicks() + 1);
  }
  else {
    motor.setOdomTicks(motor.getOdomTicks() - 1);
  }
}

//////////////////////////////////Motor Functions//////////////////////////////////////
// TODO(Josecisneros001): Check if there is a way to avoid the use of global variables 
// in this static functions.
// Robot* is declared in base_control.ino. It is used as a global variable to use it in
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
