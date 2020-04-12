
//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO() {
  bno_ = Adafruit_BNO055(55);
  
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
int BNO::getCurrentAngle() {
  int angle = 0;
  angle = getAngleX();
  angle -= bno_set_point_;
  if (angle <0) {
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