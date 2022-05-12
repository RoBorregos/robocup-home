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
