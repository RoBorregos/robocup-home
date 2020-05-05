#include <ros.h>
#include "BNO.h"
#include "Movement.h"
#include "Odometry.h"
#include "Plot.h"

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

    Odometry odom(robot,&nh);
    nh.loginfo("Odometry Initialization completed.");

    Plot plot(robot);
    nh.loginfo("Plot Initialization completed.");

    odom.run();
}

void loop() {   
}
