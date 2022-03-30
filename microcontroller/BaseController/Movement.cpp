#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno, ros::NodeHandle *nh) : bno_(bno), nh_(nh) {
  kinematics_ = Kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS, IS_OMNI);

  back_left_motor_ = Motor(MotorId::BackLeft, kDigitalPinsBackLeftMotor[1], 
                            kDigitalPinsBackLeftMotor[0], kAnalogPinBackLeftMotor, 
                            kEncoderPinsBackLeftMotor[0], kEncoderPinsBackLeftMotor[1]);
  front_left_motor_ = Motor(MotorId::FrontLeft, kDigitalPinsFrontLeftMotor[0], 
                            kDigitalPinsFrontLeftMotor[1], kAnalogPinFrontLeftMotor, 
                            kEncoderPinsFrontLeftMotor[0], kEncoderPinsFrontLeftMotor[1]);
  back_right_motor_ = Motor(MotorId::BackRight, kDigitalPinsBackRightMotor[0], 
                            kDigitalPinsBackRightMotor[1], kAnalogPinBackRightMotor, 
                            kEncoderPinsBackRightMotor[0], kEncoderPinsBackRightMotor[1]);
  front_right_motor_ = Motor(MotorId::FrontRight, kDigitalPinsFrontRightMotor[0], 
                            kDigitalPinsFrontRightMotor[1], kAnalogPinFrontRightMotor, 
                            kEncoderPinsFrontRightMotor[0], kEncoderPinsFrontRightMotor[1]);

  for(int i = 0; i < Movement::kCountMotors; i++) {
		this->last_encoder_counts_[i] = 0;
  }
}

//////////////////////////////////Encoders//////////////////////////////////////

void Movement::initEncoders() {
  back_left_motor_.initEncoders();
  front_left_motor_.initEncoders();
  back_right_motor_.initEncoders();
  front_right_motor_.initEncoders();
}

//////////////////////////////////VELOCITY//////////////////////////////////////
double constrainDa(double x, double min_, double max_) {
  if (x < min_) {
    return min_;
  }
  if (x > max_) {
    return max_;
  }
  return x;
}

void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z) {
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  
  Kinematics::output rpm = kinematics_.getRPM(x, y, z);

  if (kUsingPID) {
    front_left_motor_.setMotorSpeedPID(rpm.motor1);
    front_right_motor_.setMotorSpeedPID(rpm.motor2);
    back_left_motor_.setMotorSpeedPID(rpm.motor3);
    back_right_motor_.setMotorSpeedPID(rpm.motor4);
  } else {
    char log_msg[20];
    char result[8]; // Buffer big enough for 7-character float
    dtostrf(rpm.motor1, 6, 2, result);
    sprintf(log_msg,"M1 RPM :%s", result);
    nh_->loginfo(log_msg);

    dtostrf(rpm.motor2, 6, 2, result);
    sprintf(log_msg,"M2 RPM :%s", result);
    nh_->loginfo(log_msg);

    dtostrf(rpm.motor3, 6, 2, result);
    sprintf(log_msg,"M3 RPM :%s", result);
    nh_->loginfo(log_msg);

    dtostrf(rpm.motor4, 6, 2, result);
    sprintf(log_msg,"M4 RPM :%s", result);
    nh_->loginfo(log_msg);
  
    front_left_motor_.setMotorSpeed(rpm.motor1);
    front_right_motor_.setMotorSpeed(rpm.motor2);
    back_left_motor_.setMotorSpeed(rpm.motor3);
    back_right_motor_.setMotorSpeed(rpm.motor4);
  }
}

void Movement::stop() {
  front_left_motor_.stop();
  front_right_motor_.stop();
  back_left_motor_.stop();
  back_right_motor_.stop();
}

void Movement::getEncoderCounts(int *delta_encoder_counts) {
	delta_encoder_counts[0] = front_left_motor_.getEncoderTicks();
	delta_encoder_counts[1] = front_right_motor_.getEncoderTicks();
	delta_encoder_counts[2] = back_left_motor_.getEncoderTicks();
	delta_encoder_counts[3] = back_right_motor_.getEncoderTicks();

  front_left_motor_.setEncoderTicks(0);
  front_right_motor_.setEncoderTicks(0);
  back_left_motor_.setEncoderTicks(0);
  back_right_motor_.setEncoderTicks(0);
}
