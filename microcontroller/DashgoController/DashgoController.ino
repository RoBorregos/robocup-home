#include <ros.h>
#include "Movement.h"
#include "Odometry.h"
#include "Plot.h"

Movement *robot = nullptr;
bool ROS_ENABLE = true;
bool CHECK_PID = false;
bool CHECK_MOTORS = false;

void setup() {
    ros::NodeHandle nh;

    if (ROS_ENABLE) {
      nh.initNode();
      while (!nh.connected()) {
          nh.spinOnce();
      }
      nh.loginfo("Node Initialization completed."); 
    }
    
    Movement initRobot(&nh);
    robot = &initRobot;
    robot->initEncoders();

    if (!ROS_ENABLE) {
        loop();
    }
    
    nh.loginfo("Movement Initialization completed.");

    Plot plot(robot, !ROS_ENABLE);
    Serial3.begin(57600);
    plot.startSequence();
    nh.loginfo("Plot Initialization completed.");        

    Odometry odom(robot,&nh, &plot);
    nh.loginfo("Odometry Initialization completed.");

    odom.run();
}

// Used if ROS disabled.
void loop() {
    if (CHECK_PID) {
        Serial.begin(57600);
        // Check PID
        while(1) {
            Plot plot(robot, !ROS_ENABLE);
            delay(2000);
            plot.startSequence();
            // List of x Velocities
            double xVelocities[] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
            long long time = 0.0;
            int i = 0;
            bool sign = false;
            while(1) {
                if (millis() - time > 6000) {
                    time = millis();
                    if (sign) {
                      i++;
                      if (i == 7) {
                          i = 0;
                      }
                      sign = false;
                    }else {
                      sign = true;
                    }
                }
                robot->cmdVelocity(0.5, 0, 0, false);
                plot.plotTargetandCurrent();
            }
        }
    }
    if (CHECK_MOTORS) {
        Serial.begin(9600);
        // Check Motors
        while(1) {
          long long start = millis();
          robot->right_motor_.setPidTicks(0);
          robot->left_motor_.setPidTicks(0);
          while(millis()-start < 1000) {
              robot->right_motor_.changePwm(255);
              robot->right_motor_.forward();
              robot->left_motor_.changePwm(255);
              robot->left_motor_.forward();
          }
          robot->right_motor_.changePwm(0);
          robot->left_motor_.changePwm(0);
          Serial.print("R");
          Serial.println( (robot->right_motor_.getPidTicks() / 300) * 60 );
          Serial.print("L");
          Serial.println( (robot->left_motor_.getPidTicks() / 300) * 60 );
          delay(1000);
        }
    }
}
