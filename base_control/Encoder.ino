void handleEncoder(Motor &motor){
  motor.ticks++;
  if(motor.actualState==forward)
    motor.odomTicks++;
  else
    motor.odomTicks++;
}
void BLencoder(){
  handleEncoder(moveAll.B_left);
}
void BRencoder(){
  handleEncoder(moveAll.B_right);
}
void FLencoder(){
  handleEncoder(moveAll.F_left);
}
void FRencoder(){
  handleEncoder(moveAll.F_right);
}
