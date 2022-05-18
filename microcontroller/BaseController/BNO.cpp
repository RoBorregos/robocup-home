#include "BNO.h"

//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO() {
  
  if (!bno_.begin()) {
    while (1);
  }
  
  bno_.setExtCrystalUse(true);
  sensors_event_t event;
  bno_.getEvent(&event);
  bno_set_point_ = event.orientation.x;
}

//////////////////////////////////Calibration//////////////////////////////////////
uint8_t BNO::orientationStatus() {
  uint8_t system, gyro, accel, mag = 0;
  bno_.getCalibration(&system, &gyro, &accel, &mag);
  return mag;
}


//////////////////////////////////Get Functions//////////////////////////////////////
int BNO::getCurrentXAngle() {
  int angle = getAngleX() - bno_set_point_;
  if (angle < 0) {
    angle +=  360;
  }
  return angle;
}

double BNO::getAngleX() {
  sensors_event_t event;
  bno_.getEvent(&event);

  return event.orientation.x;
}

double BNO::getAngleY() {
  sensors_event_t event;
  bno_.getEvent(&event);

  return event.orientation.y;
}

double BNO::getAngleZ() {
  sensors_event_t event;
  bno_.getEvent(&event);

  return event.orientation.z;
}

void BNO::updateEvents(){
  bno_.getEvent(&angVelocityData_, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_.getEvent(&linearAccelData_, Adafruit_BNO055::VECTOR_LINEARACCEL);
  quat_ = bno_.getQuat();
}

float BNO::getQuat_x(){
  updateEvents();
  return quat_.x();
}

float BNO::getQuat_y(){
  updateEvents();
  return quat_.y();
}

float BNO::getQuat_z(){
  updateEvents();
  return quat_.z();
}

float BNO::getQuat_w(){
  updateEvents();
  return quat_.w();
}

float BNO::getAngVel_x(){
  updateEvents();
  return angVelocityData_.gyro.x;
}

float BNO::getAngVel_y(){
  updateEvents();
  return angVelocityData_.gyro.y;
}

float BNO::getAngVel_z(){
  updateEvents();
  return angVelocityData_.gyro.z;
}

float BNO::getLinAcc_x(){
  updateEvents();
  return linearAccelData_.acceleration.x;
}

float BNO::getLinAcc_y(){
  updateEvents();
  return linearAccelData_.acceleration.y;
}

float BNO::getLinAcc_z(){
  updateEvents();
  return linearAccelData_.acceleration.z;
}

void BNO::anglesInfo(){
  /*Serial.print(F("BNO INFO**** \n----------\nQuaternion\n  X = "));
  Serial.println((float)getQuat_x(),4);
  Serial.print(F("  Y = "));
  Serial.println((float)getQuat_y(),4);
  Serial.print(F("  Z = "));
  Serial.println((float)getQuat_z(),4);
  Serial.print(F("  w = "));
  Serial.println((float)getQuat_w(),4);
  Serial.print(F("----------\nAngular Vel\n  X = "));
  Serial.println((float)getAngVel_x(),4);
  Serial.print(F("  Y = "));
  Serial.println((float)getAngVel_y(),4);
  Serial.print(F("  Z = "));
  Serial.println((float)getAngVel_z(),4);
  Serial.print(F("----------\nLinear Acc\n  X = "));
  Serial.println((float)getLinAcc_x(),4);
  Serial.print(F("  Y = "));
  Serial.println((float)getLinAcc_y(),4);
  Serial.print(F("  Z = "));
  Serial.println((float)getLinAcc_z(),4);
  Serial.print(F("----------\nAngle Published\n  X = "));
  Serial.println((float)getAngleX(),4);
  Serial.print(F("  Y = "));
  Serial.println((float)getAngleY(),4);
  Serial.print(F("  Z = "));
  Serial.println((float)getAngleZ(),4);
  */
}
