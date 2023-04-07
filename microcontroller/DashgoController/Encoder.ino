#include "Encoder.h"
//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor, int sign) {
  int op_sign = sign == 1 ? -1 : 1;
  motor.setEncodersDir((int)(digitalRead(motor.getEncoderTwo()) == HIGH ? sign : op_sign));
  motor.setPidTicks(motor.getPidTicks() + 1);
  motor.setOdomTicks(motor.getOdomTicks() + (motor.getEncodersDir()));
}

//////////////////////////////////Motor Functions//////////////////////////////////////
void Encoder::leftEncoder() {
  handleEncoder(robot->left_motor_, -1);
}

void Encoder::rightEncoder() {
  handleEncoder(robot->right_motor_, 1);
}
