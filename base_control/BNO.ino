
BNO::BNO(){
  bno_ = Adafruit_BNO055();
  
  if (!bno_.begin()) {
    while (1);
  }
  
  bno_.setExtCrystalUse(true);
  sensors_event_t event;
  bno_.getEvent(&event);
  BNOSetPoint = event.orientation.x;

}

int BNO::actualAngle(){
  int angle = 0;
  angle = getAngleX();
  angle -= BNOSetPoint;
  if (angle <0){
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

uint8_t BNO::orientationStatus() {
  uint8_t system, gyro, accel, mag = 0;
  bno_.getCalibration(&system, &gyro, &accel, &mag);

  return mag;
}