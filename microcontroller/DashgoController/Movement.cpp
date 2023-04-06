#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(ros::NodeHandle *nh) : nh_(nh), kinematics_(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits) {
  left_motor_ = Motor(nh_, MotorId::Left, kFwdRevPinLeftMotor, 
                          kSpeedPinLeftMotor, kEnablePinLeftMotor, 
                          kEncoderPinsLeftMotor[0], kEncoderPinsLeftMotor[1], true);
  right_motor_ = Motor(nh_, MotorId::Right, kFwdRevPinRightMotor, 
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
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z, bool debug = false)
{
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);

  if (debug)
  {
    if (linear_x != x)
    {
      nh_->loginfo("Linear velocity in X constrained!");
    }
    if (linear_y != y)
    {
      nh_->loginfo("Linear velocity in Y constrained!");
    }
    if (angular_z != z)
    {
      nh_->loginfo("Angular velocity in Z constrained!");
    }
  }

  Kinematics::output rpm = kinematics_.getRPM(x, y, z);
  
  updatePIDKinematics(rpm.motor1, rpm.motor2);

  if (debug)
  {
    char log_msg[20];
    char result[8];
    double rpms[kMotorCount];
    rpms[0] = (float)rpm.motor1; // Left
    rpms[1] = (float)rpm.motor2; // Right

    dtostrf(angular_z, 6, 2, result);
    sprintf(log_msg, "Angular Z :%s", result);
    nh_->loginfo(log_msg);
    dtostrf(linear_x, 6, 2, result);
    sprintf(log_msg, "Linear X :%s", result);
    nh_->loginfo(log_msg);
    dtostrf(linear_y, 6, 2, result);
    sprintf(log_msg, "Linear Y :%s", result);
    nh_->loginfo(log_msg);
    dtostrf(millis() - cycle, 6, 2, result);
    sprintf(log_msg, "Cycle :%s", result);
    nh_->loginfo(log_msg);
    cycle = millis();
  }
}

void Movement::updatePIDKinematics(double rm_speed, double lm_speed) {
  right_motor_.constantRPM(rm_speed);
  left_motor_.constantRPM(lm_speed);
}
