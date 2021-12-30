#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno, ros::NodeHandle *nh) : bno_(bno), nh_(nh) {

  back_left_motor_ = Motor(MotorId::BackLeft, kDigitalPinsBackLeftMotor[0], 
                            kDigitalPinsBackLeftMotor[1], kAnalogPinBackLeftMotor, 
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
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z) {
  double x = constrain(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrain(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrain(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  double kMotorMaxV = Motor::kMaxVelocity;

	double front_left = x - y - z;
  double front_right = x + y + z;
  double back_left = x + y - z;
  double back_right = x - y + z;

  // Weighted sum if out of range.
  if (front_left > kMotorMaxV || front_left < kMotorMaxV) {
    front_left = (x * kMotorMaxV / front_left) - (y * kMotorMaxV / front_left) - (z * kMotorMaxV / front_left);
  }
  if (front_right > kMotorMaxV || front_right < kMotorMaxV) {
    front_right = (x * kMotorMaxV / front_right) + (y * kMotorMaxV / front_right) + (z * kMotorMaxV / front_right);
  }
  if (back_left > kMotorMaxV || back_left < kMotorMaxV) {
    back_left = (x * kMotorMaxV / back_left) + (y * kMotorMaxV / back_left) - (z * kMotorMaxV / back_left);
  }
  if (back_right > kMotorMaxV || back_right < kMotorMaxV) {
    back_right = (x * kMotorMaxV / back_right) - (y * kMotorMaxV / back_right) + (z * kMotorMaxV / back_right);
  }
	
  if (kUsingPID) {
    front_left_motor_.setMotorSpeedPID(front_left);
    front_right_motor_.setMotorSpeedPID(front_right);
    back_left_motor_.setMotorSpeedPID(back_left);
    back_right_motor_.setMotorSpeedPID(back_right);
  } else {
    front_left_motor_.setMotorSpeed(front_left);
    front_right_motor_.setMotorSpeed(front_right);
    back_left_motor_.setMotorSpeed(back_left);
    back_right_motor_.setMotorSpeed(back_right);
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
