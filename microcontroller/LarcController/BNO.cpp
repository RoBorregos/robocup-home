#include "BNO.h"

//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO() {
  bno_ = Adafruit_BNO055(55);
  if (!bno_.begin()) {
    return;
  }
  
  bno_.setExtCrystalUse(true);
  sensors_event_t event;
  bno_.getEvent(&event);

}

//////////////////////////////////Calibration//////////////////////////////////////
uint8_t BNO::orientationStatus() {
  if (!bno_.begin()) {
    return 0;
  }
  uint8_t system, gyro, accel, mag = 0;
  bno_.getCalibration(&system, &gyro, &accel, &mag);

  return mag;
}

void BNO::updateBNO() {
  if (!bno_.begin()) {
    return;
  }
  static long long last_time = 0;
  static sensors_event_t orientationData , angVelocityData, accelerometerData;
  if (millis() - last_time > 50) {
    last_time = millis();
    bno_.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno_.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno_.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    yaw_ = orientationData.orientation.x;
    yaw_vel_ = angVelocityData.gyro.x;
    x_accel = accelerometerData.acceleration.x;
    y_accel = accelerometerData.acceleration.y;
    z_accel = accelerometerData.acceleration.z;
  }
}

float BNO::getYaw() {
  updateBNO();
  return yaw_;
}
float BNO::getYawVel() {
  updateBNO();
  return yaw_vel_;
}
float BNO::getXAccel() {
  updateBNO();
  return x_accel;
}
float BNO::getYAccel() {
  updateBNO();
  return y_accel;
}
float BNO::getZAccel() {
  updateBNO();
  return z_accel;
}

//////////////////////////////////Get Functions//////////////////////////////////////



void BNO::reset() {
  digitalWrite(reset_pin_, LOW);  
  delayMicroseconds(30);
  digitalWrite(reset_pin_, HIGH);
  bno_ = Adafruit_BNO055(55);
}
