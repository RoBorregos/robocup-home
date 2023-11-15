#include "BNO.h"

//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO() {
  bno_ = Adafruit_BNO055(55);
  if (!bno_.begin()) {
    return;
  }
  Serial.begin(57600);
  bno_.setExtCrystalUse(true);
  sensors_event_t event;
  bno_.getEvent(&event);
}

//////////////////////////////////Calibration//////////////////////////////////////
uint8_t BNO::orientationStatus() {
  uint8_t system, gyro, accel, mag = 0;
  bno_.getCalibration(&system, &gyro, &accel, &mag);

  return mag;
}

void BNO::updateBNO() {
  static long long last_time = 0;
  static sensors_event_t orientationData , angVelocityData, accelerometerData;
  if (millis() - last_time > 10) {
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
  return yaw_;
}
float BNO::getYawVel() {
  return yaw_vel_;
}
float BNO::getXAccel() {
  return x_accel;
}
float BNO::getYAccel() {
  return y_accel;
}
float BNO::getZAccel() {
  return z_accel;
}

//////////////////////////////////Get Functions//////////////////////////////////////



void BNO::reset() {
  digitalWrite(reset_pin_, LOW);  
  delayMicroseconds(30);
  digitalWrite(reset_pin_, HIGH);
  bno_ = Adafruit_BNO055(55);
}
