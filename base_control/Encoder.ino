//////////////////////////////////Main Function//////////////////////////////////////
void Encoder::handleEncoder(Motor &motor){
  motor.setPidTicks(motor.getPidTicks() + 1);
  if(motor.getCurrentState() == Forward) {
    motor.setOdomTicks(motor.getOdomTicks() + 1);
  }
  else {
    motor.setOdomTicks(motor.getOdomTicks() - 1);
  }
}

//////////////////////////////////Motor Functions//////////////////////////////////////
void Encoder::backLeftEncoder(){
  handleEncoder(robot->back_left_motor_);
}
void Encoder::backRightEncoder(){
  handleEncoder(robot->back_right_motor_);
}
void Encoder::frontLeftEncoder(){
  handleEncoder(robot->front_left_motor_);
}
void Encoder::frontRightEncoder(){
  handleEncoder(robot->front_right_motor_);
}
