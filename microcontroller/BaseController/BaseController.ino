#include <ros.h>
#include "BNO.h"
#include "Movement.h"
#include "RosBridge.h"

Movement *robot = nullptr;

void setup() {
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

    RosBridge rosbridge(robot, &nh);
    nh.loginfo("RosBridge Initialization completed.");

    rosbridge.run();
}

void loop() {   
}
