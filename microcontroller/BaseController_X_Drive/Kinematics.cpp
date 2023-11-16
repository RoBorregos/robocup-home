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
  circumference_ = PI * wheel_diameter;
  max_rpm_ = motor_max_rpm;
  fr_wheels_dist_ = fr_wheels_dist;
  lr_wheels_dist_ = lr_wheels_dist;
  pwm_res_ = pow(2, pwm_bits) - 1;
  this->bno = bno; //Pass BNO info by reference
}


Kinematics::output Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  target_angle = 0;
  float direction_angle = 90;
  //Get current angle using BNO Yaw
  float curr_angle_x = bno->getYaw();
  
  //float curr_angle_x = 0;
  float angle_diff = curr_angle_x - past_angle;
  //Distance from the center of the robot to the center of the wheels
  float R = lr_wheels_dist_;
  float target_v_rot = 100000;;
  /*
  if(curr_angle_x-target_angle > 5){
    target_v_rot = 0.5;
  }
  else if(curr_angle_x-target_angle < 5){
    target_v_rot = -0.5;
  }
  else{
    target_v_rot = 0;
  }

  if(target_v_rot>angle_diff){
    
  }
  */


  
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

  
/*
     //OLD KINEMATICS
    //front-left motor
    rpm.motor1 = x_rpm_ - y_rpm_ - tan_rpm_;
    //rear-left motor
    rpm.motor3 = x_rpm_ + y_rpm_ - tan_rpm_;

    //front-right motor
    rpm.motor2 = x_rpm_ + y_rpm_ + tan_rpm_;
    //rear-right motor
    rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;

    */

    ///////////Test kinematics//////////////////////
    //NEW KINEMATICS
      //Hacer caso a esta cosa 
      //Front left es el motor que está marcado como 2 en el robot y como rpm.motor1 en las funciones
      //Front right es el motor que est+a marcado como 1 en el robot y como rpm.motor2 en las funciones
      //Back left es el motor que está marcado como 3 en el robot y como rpm.motor3 en las funciones
      //Back right es el motor que está marcado como 4 en el robot y como rpm.motor4 en las funciones
            
      /*       CORRECTED KINEMATICS 
      //front-right motor
      rpm.motor2 = (-1*sin(PI/4+curr_angle_x*(PI/180))*0 + cos(PI/4+curr_angle_x*(PI/180))*5 + R*10)/(circumference_ /(2*PI)); //R*theta is the rotational speed of the robot, which is calculated using R as the distance from the wheels to the center of the robot, and theta as the angular displacement in radians
      //front-left motor
      rpm.motor1 = (-1*sin(3*PI/4+curr_angle_x*(PI/180))*0 + cos(3*PI/4+curr_angle_x*(PI/180))*5 + R*10)/(circumference_/(2*PI));
      //back-left motor
      rpm.motor3 = (-1*sin(5*PI/4+curr_angle_x*(PI/180))*0 + cos(5*PI/4+curr_angle_x*(PI/180))*5 + R*10)/(circumference_/(2*PI));
      //back-right motor
      rpm.motor4 = (-1*sin(7*PI/4+curr_angle_x*(PI/180))*0 + cos(7*PI/4+curr_angle_x*(PI/180))*5  + R*10)/(circumference_/(2*PI));
      */ 

      

      //front-right motor
/*
      if((curr_angle_x-target_angle > 10){
        rpm.motor2 = (-0*sin(1*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(1*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI)); //R*theta is the rotational speed of the robot, which is calculated using R as the distance from the wheels to the center of the robot, and theta as the angular displacement in radians
        //front-left motor
        rpm.motor1 = (-0*sin(3*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(3*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
        //back-left motor
        rpm.motor3 = (-0*sin(5*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(5*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
        //back-right motor
        rpm.motor4 = (-0*sin(7*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(7*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
      }
      else if((curr_angle_x > 5+target_angle && curr_angle_x < 355)){
        rpm.motor2 = (-0*sin(1*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(1*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI)); //R*theta is the rotational speed of the robot, which is calculated using R as the distance from the wheels to the center of the robot, and theta as the angular displacement in radians
        //front-left motor
        rpm.motor1 = (-0*sin(3*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(3*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
        //back-left motor
        rpm.motor3 = (-0*sin(5*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(5*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
        //back-right motor
        rpm.motor4 = (-0*sin(7*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + 0*cos(7*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*10)/(circumference_/(2*PI));
      }
      else{
      */
        rpm.motor2 = (-1*sin(1*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + cos(1*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*0)/(circumference_/(2*PI)); //R*theta is the rotational speed of the robot, which is calculated using R as the distance from the wheels to the center of the robot, and theta as the angular displacement in radians
        //front-left motor
        rpm.motor1 = (-1*sin(3*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + cos(3*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*0)/(circumference_/(2*PI));
        //back-left motor
        rpm.motor3 = (-1*sin(5*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + cos(5*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*0)/(circumference_/(2*PI));
        //back-right motor
        rpm.motor4 = (-1*sin(7*PI/4+curr_angle_x*(PI/180))*(5*(cos(PI/180 * (direction_angle)))) + cos(7*PI/4+curr_angle_x*(PI/180))*5*(sin(PI/180 * (direction_angle))) + R*0)/(circumference_/(2*PI));
      
      Serial.println(rpm.motor1);
      Serial.println(rpm.motor2);
      Serial.println(rpm.motor3);
      Serial.println(rpm.motor4);
      Serial.println(curr_angle_x);

      //float angle_2 = -PI/3; 


      /*
      //FRONT RIGHT 
      rpm.motor2 = cos(angle_2-PI/4)*10000; 
      //front-left motor
      rpm.motor1 = cos(angle_2+PI/4)*10000;
      //back-left motor
      rpm.motor3 = -cos(angle_2-PI/4)*10000; 
      //back-right motor
      rpm.motor4 = -cos(angle_2+PI/4)*10000; 
      */
      
    ////////////////////////////////////////////////
    //NEW KINEMATICS
    /*
    //front-left motor
    rpm.motor1 = (-1*sin(135)*x_rpm_ + cos(135)*y_rpm_ + R*angle_diff)/(circumference_ / (2*PI)); //R*theta is the rotational speed of the robot, which is calculated using R as the distance from the wheels to the center of the robot, and theta as the angular displacement in radians
    //rear-left motor
    rpm.motor3 = (-1*sin(225)*x_rpm_+cos(225)*y_rpm_+R*angle_diff)/(circumference_/(2*PI));
    //front-right motor
    rpm.motor2 = (-1*sin(45)*x_rpm_+cos(45)*y_rpm_+R*angle_diff)/(circumference_/(2*PI));
    //rear-right motor
    rpm.motor4 = (-1*sin(315)*x_rpm_+cos(315)*y_rpm_+R*angle_diff)/(circumference_/(2*PI));
    */
    
    past_angle = curr_angle_x;
    bno->updateBNO();
    
  
  return rpm;
}

Kinematics::output Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
  Kinematics::output rpm;
  Kinematics::output pwm;

  rpm = getRPM(linear_x, linear_y, angular_z);

  //convert from RPM to PWM
  //front-left motor
  pwm.motor1 = rpmToPWM(rpm.motor1);
  //rear-left motor
  pwm.motor2 = rpmToPWM(rpm.motor2);

  //front-right motor
  pwm.motor3 = rpmToPWM(rpm.motor3);
  //rear-right motor
  pwm.motor4 = rpmToPWM(rpm.motor4);

  return pwm;
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
  return (((float) rpm / (float) max_rpm_) * pwm_res_);
}
