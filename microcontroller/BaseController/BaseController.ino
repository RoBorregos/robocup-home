#include <ros.h>
#include "BNO.h"
#include "Movement.h"
#include "RosBridge.h"
#include "Plot.h"

Movement *robot = nullptr;
unsigned long oldtime;

void setup() {
    Serial.begin(57600);
    /*
    BNO bno;
    ros::NodeHandle nh; 
    nh.loginfo("Bno Initialization completed.");
    Movement initRobot(&bno, &nh);
    robot = &initRobot;
    robot->initEncoders();
    nh.loginfo("Movement Initialization completed.");

    oldtime=millis();
    Plot myPlot(robot);
    while(oldtime <= 6000){
      oldtime = millis();
      robot->back_left_motor_.setMotorSpeedPID(0.13);
      robot->front_left_motor_.setMotorSpeedPID(0.13);
      robot->back_right_motor_.setMotorSpeedPID(0.13);
      robot->front_right_motor_.setMotorSpeedPID(0.13);
      myPlot.PlotTargetandCurrent();
    }
    while(oldtime > 6000 && oldtime <= 10000){
      oldtime = millis();
      robot->back_left_motor_.setMotorSpeedPID(0.17);
      robot->front_left_motor_.setMotorSpeedPID(0.17);
      robot->back_right_motor_.setMotorSpeedPID(0.17);
      robot->front_right_motor_.setMotorSpeedPID(0.17);
      myPlot.PlotTargetandCurrent();
    }
    while(oldtime > 10000 && oldtime <= 14000){
      oldtime = millis();
      robot->back_left_motor_.setMotorSpeedPID(0.20);
      robot->front_left_motor_.setMotorSpeedPID(0.20);
      robot->back_right_motor_.setMotorSpeedPID(0.20);
      robot->front_right_motor_.setMotorSpeedPID(0.20);
      myPlot.PlotTargetandCurrent();
    }
    robot->back_left_motor_.setMotorSpeedPID(0);
    robot->front_left_motor_.setMotorSpeedPID(0);
    robot->back_right_motor_.setMotorSpeedPID(0);
    robot->front_right_motor_.setMotorSpeedPID(0);
    
    return;
    */
    /*
    Serial.begin(9600);
    
    Motor back_left_motor_ = Motor(MotorId::BackLeft, Movement::kDigitalPinsBackLeftMotor[1], 
                            Movement::kDigitalPinsBackLeftMotor[0], Movement::kAnalogPinBackLeftMotor, 
                              Movement::kEncoderPinsBackLeftMotor[0], Movement::kEncoderPinsBackLeftMotor[1]);
    Motor front_left_motor_ = Motor(MotorId::FrontLeft, Movement::kDigitalPinsFrontLeftMotor[0], 
                              Movement::kDigitalPinsFrontLeftMotor[1], Movement::kAnalogPinFrontLeftMotor, 
                              Movement::kEncoderPinsFrontLeftMotor[0], Movement::kEncoderPinsFrontLeftMotor[1]);
    Motor back_right_motor_ = Motor(MotorId::BackRight, Movement::kDigitalPinsBackRightMotor[0], 
                              Movement::kDigitalPinsBackRightMotor[1], Movement::kAnalogPinBackRightMotor, 
                              Movement::kEncoderPinsBackRightMotor[0], Movement::kEncoderPinsBackRightMotor[1]);
    Motor front_right_motor_ = Motor(MotorId::FrontRight, Movement::kDigitalPinsFrontRightMotor[0], 
                              Movement::kDigitalPinsFrontRightMotor[1], Movement::kAnalogPinFrontRightMotor, 
                              Movement::kEncoderPinsFrontRightMotor[0], Movement::kEncoderPinsFrontRightMotor[1]);
    back_left_motor_.setMotorSpeed(-0.18);
    front_left_motor_.setMotorSpeed(-0.18);
    back_right_motor_.setMotorSpeed(-0.18);
    front_right_motor_.setMotorSpeed(-0.18);
    return;
    */
    
    ros::NodeHandle nh; 
    
    nh.initNode();
    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("Node Initialization completed.");

    BNO bno;
    nh.loginfo("Bno Initialization completed.");
    Movement initRobot(&bno, &nh);
    robot = &initRobot;
    robot->initEncoders();
    nh.loginfo("Movement Initialization completed.");

    Plot plot(robot);
    nh.loginfo("Plot Initialization completed.");

    RosBridge rosbridge(robot, &nh);
    nh.loginfo("RosBridge Initialization completed.");

    rosbridge.run();
}

void loop() {   
}
