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
double Movement::convertToPWM(float value, const float maxValue) {
  value = constrain(value, -1.0 * maxValue, maxValue);
  double pwmValue = map(value, -1.0 * maxValue, maxValue, -1.0 * kMaxPWM, kMaxPWM);

  // Enforce deadzone around 0.
  if(abs(pwmValue) <= kDeadZone) {
    pwmValue = 0;
  }
  
  return pwmValue;
}

void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z) {
  double x = convertToPWM(linear_x, Movement::kLinearXMaxVelocity);
  double y = convertToPWM(linear_y, Movement::kLinearYMaxVelocity);
  double z = convertToPWM(angular_z, Movement::kAngularZMaxVelocity);

	double sum = abs(x) + abs(y) + abs(z);
	
	if(sum > kMaxPWM) {
		x = (x * kMaxPWM / sum);
		y = (y * kMaxPWM / sum);
		z = (z * kMaxPWM / sum);
	}
	
	front_left_motor_.setMotorSpeed(x - y - z);
	front_right_motor_.setMotorSpeed(x + y + z);
	back_left_motor_.setMotorSpeed(x + y - z);
	back_right_motor_.setMotorSpeed(x - y + z);
}

void Movement::stop() {
  front_left_motor_.stop();
  front_right_motor_.stop();
  back_left_motor_.stop();
  back_right_motor_.stop();
}

void Movement::getEncoderCounts(float *xytCounts) {
	int new_encoder_counts[Movement::kCountMotors];
	new_encoder_counts[0] = front_left_motor_.getEncoderTicks();
	new_encoder_counts[1] = front_right_motor_.getEncoderTicks();
	new_encoder_counts[2] = back_left_motor_.getEncoderTicks();
	new_encoder_counts[3] = back_right_motor_.getEncoderTicks();
	
	// find deltas
	int delta_encoder_counts[Movement::kCountMotors];
	for(int i = 0; i < Movement::kCountMotors; ++i) {
		// check for overflow
		if(abs(last_encoder_counts_[i]) > kCountOverflow && 
           abs(new_encoder_counts[i]) > kCountOverflow && 
           sign(last_encoder_counts_[i]) != sign(new_encoder_counts[i])) {

			if(sign(last_encoder_counts_[i]) > 0) {
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] + kIntMax;
            } else {
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] - kIntMax;
            }

		} else {
            delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i];
        } 
		
		if(abs(delta_encoder_counts[i]) > kCountReset) {
            delta_encoder_counts[i] = 0;
        } 
		
		last_encoder_counts_[i] = new_encoder_counts[i];
	}
  
  // convert the motor counts into x, y, theta counts
	xytCounts[0] = (delta_encoder_counts[0] + delta_encoder_counts[1] + delta_encoder_counts[2] + delta_encoder_counts[3]) / Movement::kCountMotors;
	xytCounts[1] = (0 - delta_encoder_counts[0] + delta_encoder_counts[1] + delta_encoder_counts[2] - delta_encoder_counts[3]) / Movement::kCountMotors;
	xytCounts[2] = (0 - delta_encoder_counts[0] + delta_encoder_counts[1] - delta_encoder_counts[2] + delta_encoder_counts[3]) / Movement::kCountMotors;

}
