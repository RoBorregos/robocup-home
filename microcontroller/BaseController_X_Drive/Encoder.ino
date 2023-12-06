#include "Encoder.h"
//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor, int sign = 1) {
  int op_sign = sign == 1 ? -1 : 1;
  motor.setEncodersDir((int)(digitalRead(motor.getEncoderTwo()) == HIGH ? sign : op_sign));
  motor.setPidTicks(motor.getPidTicks() + 1);
  motor.setOdomTicks(motor.getOdomTicks() + (motor.getEncodersDir()));
}

//////////////////////////////////Motor Functions//////////////////////////////////////
void Encoder::frontLeftEncoder() {
  handleEncoder(robot->back_left_motor_, 1);
}

void Encoder::backRightEncoder() {
  handleEncoder(robot->back_right_motor_, -1);
}

void Encoder::backLeftEncoder() {
  handleEncoder(robot->front_left_motor_, 1);
}

void Encoder::frontRightEncoder() {
  handleEncoder(robot->front_right_motor_, -1);
}
