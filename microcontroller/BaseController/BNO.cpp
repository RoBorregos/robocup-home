#include "BNO.h"

//////////////////////////////////Constructor//////////////////////////////////////
BNO::BNO(ros::NodeHandle *nh) : nh_(nh) {
  if (!bno_.begin()) {
    while (1);
  }
  
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


//////////////////////////////////Get Functions//////////////////////////////////////
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

sensor_msgs::Imu BNO::getImuInfo(){
  sensor_msgs::Imu bno_sensor_;
  updateEvents();
  bno_sensor_.header.frame_id = "imu_link";
  bno_sensor_.header.stamp = nh_->now(); 
  bno_sensor_.orientation.x = quat_.x();
  bno_sensor_.orientation.y = quat_.y();
  bno_sensor_.orientation.z = quat_.z();
  bno_sensor_.orientation.w = quat_.w();
  bno_sensor_.linear_acceleration.x = linearAccelData_.acceleration.x;
  bno_sensor_.linear_acceleration.y = linearAccelData_.acceleration.y;
  bno_sensor_.linear_acceleration.z = linearAccelData_.acceleration.z;

  bno_sensor_.angular_velocity.x = angVelocityData_.gyro.x;
  bno_sensor_.angular_velocity.y = angVelocityData_.gyro.y;
  bno_sensor_.angular_velocity.z = angVelocityData_.gyro.z;

  return bno_sensor_;
}
