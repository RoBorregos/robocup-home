/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

//TEST
#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist, float lr_wheels_dist, int pwm_bits, BNO *bno)
{
  //this->motor = motor;
  circumference_ = PI * wheel_diameter;
  max_rpm_ = motor_max_rpm;
  fr_wheels_dist_ = fr_wheels_dist;
  lr_wheels_dist_ = lr_wheels_dist;
  pwm_res_ = pow(2, pwm_bits) - 1;
  this->bno = bno; //Pass BNO info by reference
}



Kinematics::output Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  
  //Distance from the center of the robot to the center of the wheels
  float R = 0.33;

  //convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  // //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * lr_wheels_dist_;

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;

  // Need to check motor order //*(40/13.1947)
  rpm.motor2 = (-1*sin(1*(PI/4))*linear_vel_x_mins_+cos(1*PI/4)*linear_vel_y_mins_+angular_vel_z_mins_)*(40/11.9883);
  rpm.motor1 = (-1*sin(3*(PI/4))*linear_vel_x_mins_+cos(3*PI/4)*linear_vel_y_mins_+angular_vel_z_mins_)*(40/11.9883);
  rpm.motor3 = (-1*sin(5*(PI/4))*linear_vel_x_mins_+cos(5*PI/4)*linear_vel_y_mins_+angular_vel_z_mins_)*(40/11.9883);
  rpm.motor4 = (-1*sin(7*(PI/4))*linear_vel_x_mins_+cos(7*PI/4)*linear_vel_y_mins_+angular_vel_z_mins_)*(40/11.9883);

  return rpm;
  
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2)
{
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * circumference_); // m/s

  float average_rpm_a = (float)(motor2 - motor1) / 2;
  //convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * circumference_) / (lr_wheels_dist_ /  2);

  return vel;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2, int motor3, int motor4)
{
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2 + motor3 + motor4) / 4; // RPM
  //convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * circumference_); // m/s

  float average_rpm_y = (float)(-motor1 + motor2 + motor3 - motor4) / 4; // RPM
  //convert revolutions per minute in y axis to revolutions per second
  float average_rps_y = average_rpm_y / 60; // RPS
  vel.linear_y = (average_rps_y * circumference_); // m/s

  float average_rpm_a = (float)(-motor1 + motor2 - motor3 + motor4) / 4;
  //convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * circumference_) / ((fr_wheels_dist_ / 2) + (lr_wheels_dist_ / 2));
  
  return vel;
  
}

int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
  return (((float) rpm / (float) max_rpm_) * 255);
}
