#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement() : kinematics_(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits) {
  left_motor_ = Motor(MotorId::Left, kFwdRevPinLeftMotor, 
                          kSpeedPinLeftMotor, kEnablePinLeftMotor, 
                          kEncoderPinsLeftMotor[0], kEncoderPinsLeftMotor[1], true);
  right_motor_ = Motor(MotorId::Right, kFwdRevPinRightMotor, 
                            kSpeedPinRightMotor, kEnablePinRightMotor, 
                            kEncoderPinsRightMotor[0], kEncoderPinsRightMotor[1]);
}

//////////////////////////////////Encoders//////////////////////////////////////

void Movement::initEncoders() {
  left_motor_.initEncoders();
  right_motor_.initEncoders();
}

//////////////////////////////////PWM//////////////////////////////////////

void Movement::changePwm(const uint8_t pwm) {
  left_motor_.changePwm(pwm);
  right_motor_.changePwm(pwm);
}


//////////////////////////////////VELOCITY//////////////////////////////////////

void Movement::setDeltaX(const double delta_x) {
  delta_x_ = delta_x;
}

void Movement::setDeltaY(const double delta_y) {
  delta_y_ = delta_y;
}

void Movement::setDeltaAngular(const double delta_angular) {
  delta_angular_ = delta_angular;
}

double Movement::getDeltaX(){
  return delta_x_;
}

double Movement::getDeltaY(){
  return delta_y_;
}

double Movement::getDeltaAngular(){
  return delta_angular_;
}


double Movement::radiansToDegrees(const double radians) {
  return radians * 180 / M_PI;
}

void Movement::stop() {
  left_motor_.stop();
  right_motor_.stop();
}

// returns the constrained velocity of the robot.
double constrainDa(double x, double min_, double max_)
{
  return max(min_, min(x, max_));
}

//////////////////////////////////PID//////////////////////////////////////
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z)
{
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);

  Kinematics::output rpm = kinematics_.getRPM(x, y, -1*z);
  
  updatePIDKinematics(rpm.motor1, rpm.motor2);
}

void Movement::updatePIDKinematics(double rm_speed, double lm_speed) {
  right_motor_.constantRPM(rm_speed);
  left_motor_.constantRPM(lm_speed);
}
