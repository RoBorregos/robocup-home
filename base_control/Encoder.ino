void Encoder::handleEncoder(Motor &motor){
  motor.ticks++;
  if(motor.actualState==forward)
    motor.odomTicks++;
  else
    motor.odomTicks++;
}
void Encoder::BLencoder(){
  handleEncoder(robot->B_left);
}
void Encoder::BRencoder(){
  handleEncoder(robot->B_right);
}
void Encoder::FLencoder(){
  handleEncoder(robot->F_left);
}
void Encoder::FRencoder(){
  handleEncoder(robot->F_right);
}
