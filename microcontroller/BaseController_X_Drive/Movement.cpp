#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno) : kinematics_(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits,  bno)
{ 
  back_left_motor_ = Motor(MotorId::BackLeft, kDigitalPinsBackLeftMotor[1], 
                          kDigitalPinsBackLeftMotor[0], kAnalogPinBackLeftMotor, 
                          kEncoderPinsBackLeftMotor[0], kEncoderPinsBackLeftMotor[1]);
  front_left_motor_ = Motor(MotorId::FrontLeft, kDigitalPinsFrontLeftMotor[1], 
                            kDigitalPinsFrontLeftMotor[0], kAnalogPinFrontLeftMotor, 
                            kEncoderPinsFrontLeftMotor[0], kEncoderPinsFrontLeftMotor[1]);
  back_right_motor_ = Motor(MotorId::BackRight, kDigitalPinsBackRightMotor[0], 
                            kDigitalPinsBackRightMotor[1], kAnalogPinBackRightMotor, 
                            kEncoderPinsBackRightMotor[0], kEncoderPinsBackRightMotor[1]);
  front_right_motor_ = Motor(MotorId::FrontRight, kDigitalPinsFrontRightMotor[0], 
                            kDigitalPinsFrontRightMotor[1], kAnalogPinFrontRightMotor, 
                            kEncoderPinsFrontRightMotor[0], kEncoderPinsFrontRightMotor[1]);
}

//////////////////////////////////Encoders//////////////////////////////////////

void Movement::initEncoders() {
  back_left_motor_.initEncoders();
  front_left_motor_.initEncoders();
  back_right_motor_.initEncoders();
  front_right_motor_.initEncoders();
}

//////////////////////////////////PWM//////////////////////////////////////

void Movement::changePwm(const uint8_t pwm) {
  back_left_motor_.changePwm(pwm);
  front_left_motor_.changePwm(pwm);
  back_right_motor_.changePwm(pwm);
  front_right_motor_.changePwm(pwm);
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
  back_left_motor_.stop();
  front_left_motor_.stop();
  back_right_motor_.stop();
  front_right_motor_.stop();

}

// returns the constrained velocity of the robot.
double constrainDa(double x, double min_, double max_)
{
  return max(min_, min(x, max_));
}


///////////////////////////Auxiliar function to test kinematics (Linear + Traslational)///////////////////////
void Movement::cmdVelocityKinematics(const double linear_x, const double linear_y, const double angular_z){
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  Kinematics::output pwm = kinematics_.getPWM(x, y, z);
  updatePIDKinematics(pwm.motor1, pwm.motor2, pwm.motor3, pwm.motor4);

}

//////////////////////////////////PID//////////////////////////////////////
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z)
{
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  Kinematics::output rpm = kinematics_.getRPM(x, y, z);
  updatePIDKinematics(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);
}



void Movement::updatePIDKinematics(double fr_speed, double fl_speed, double bl_speed, double br_speed) {
  front_left_motor_.constantRPM(fr_speed);
  front_right_motor_.constantRPM(fl_speed);
  back_left_motor_.constantRPM(bl_speed);
  back_right_motor_.constantRPM(br_speed);
}
