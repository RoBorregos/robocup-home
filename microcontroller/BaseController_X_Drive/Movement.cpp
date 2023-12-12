#include "Movement.h"

//////////////////////////////////Constructor//////////////////////////////////////
Movement::Movement(BNO *bno) : kinematics_(kRPM, kWheelDiameter, kFrWheelsDist, kLrWheelsDist, kPwmBits,  bno)
{ 
  pidBno = PID(kBnoKP, kBnoKI, kBnoKD, 0, 360, kMaxErrorSum, 100);

  this->bno = bno;

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

////////////////////////////SET A DESIRED ROBOT ANGLE//////////////////////////////
void Movement::setRobotAngle(const double angle){
  robotAngle = angle;
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

//Returns the constrained velocity of the robot.
double constrainDa(double x, double min_, double max_)
{
  return max(min_, min(x, max_));
}

//////////////////////////////////PID//////////////////////////////////////
void Movement::cmdVelocity(const double linear_x, const double linear_y, const double angular_z){
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  Kinematics::output rpm = kinematics_.getRPM(x, y, z);
  updatePIDKinematics(rpm.motor2, rpm.motor1, rpm.motor3, rpm.motor4);

  Serial.print("Back Left Motor Setpoint RPM: "); Serial.println(rpm.motor1);
  Serial.print("Back Right Motor Setpoint RPM: "); Serial.println(rpm.motor2);
  Serial.print("Front Left Motor Setpoint RPM: "); Serial.println(rpm.motor3);
  Serial.print("Front Right Motor Setpoint RPM: "); Serial.println(rpm.motor4);  
  Serial.print("Back Left Motor PWM: "); Serial.println(back_left_motor_.getPWM());
  Serial.print("Back Right Motor PWM: "); Serial.println(back_right_motor_.getPWM());
  Serial.print("Front Left Motor PWM: "); Serial.println(front_left_motor_.getPWM());
  Serial.print("Front Right Motor PWM: "); Serial.println(front_right_motor_.getPWM());
}

//////////////////////////ADJUSTING CMD VELOCITY BASED ON BNO FEEDBACK//////////////////////////
void Movement::orientedMovement(double linear_x, double linear_y, double angular_z){
  bno->updateBNO();
  float current_angle = bno->getYaw();
  /*
  if(angular_z != 0){
      robotAngle = current_angle;
  }
  else{
    angle_error = fabs(current_angle - robotAngle);
    if(angle_error > kAngleTolerance){
        float angularSpeed = pidBno.compute_dt(robotAngle, current_angle, kBNO_time);
        angular_z = (current_angle > 2 && current_angle < 180) ? -angularSpeed : angularSpeed;
    }
  }
  */
  double x = constrainDa(linear_x, -1.0 * kLinearXMaxVelocity, kLinearXMaxVelocity);
  double y = constrainDa(linear_y, -1.0 * kLinearYMaxVelocity, kLinearYMaxVelocity);
  double z = constrainDa(angular_z, -1.0 * kAngularZMaxVelocity, kAngularZMaxVelocity);
  Kinematics::output rpm = kinematics_.getRPM(x, y, z);
  updatePIDKinematics(rpm.motor2, rpm.motor1, rpm.motor3, rpm.motor4);
  
  Serial.println(" ");
  Serial.println("//////////////////");
  Serial.print("Goal Angle:"); Serial.println(robotAngle);
  Serial.print("Angle Error:"); Serial.println(angle_error);
  Serial.print("Anugular Speed:"); Serial.println(angular_z);
  Serial.println(" ");
  Serial.println("//////////////////");
  Serial.println(" ");
  Serial.print("Back Left Motor Setpoint RPM: "); Serial.println(rpm.motor1);
  Serial.print("Back Right Motor Setpoint RPM: "); Serial.println(rpm.motor2);
  Serial.print("Front Left Motor Setpoint RPM: "); Serial.println(rpm.motor3);
  Serial.print("Front Right Motor Setpoint RPM: "); Serial.println(rpm.motor4);
  Serial.println("//////////////////");
  Serial.println(" ");  
}

void Movement::updatePIDKinematics(double fl_speed, double fr_speed, double bl_speed, double br_speed) {
  front_left_motor_.stableRPM(fr_speed);
  front_right_motor_.stableRPM(fl_speed);
  back_left_motor_.stableRPM(bl_speed);
  back_right_motor_.stableRPM(br_speed);
}
