#include "Encoder.h"
//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor) {
  // TODO FIX Direction
  motor.setEncodersDir(
    digitalRead(motor.getEncoderOne()) == 1 && digitalRead(motor.getEncoderTwo() == 0) ? 1 : -1
  );
  motor.setPidTicks(motor.getPidTicks() + 1);
  if(motor.getCurrentState() == MotorState::Forward) {
    motor.setOdomTicks(motor.getOdomTicks() + 1);
  }
  else {
    motor.setOdomTicks(motor.getOdomTicks() - 1);
  }
}

//////////////////////////////////Motor Functions//////////////////////////////////////
void Encoder::leftEncoder() {
  handleEncoder(robot->left_motor_);
}

void Encoder::rightEncoder() {
  handleEncoder(robot->right_motor_);
}
