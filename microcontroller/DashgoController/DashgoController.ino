#include "Movement.h"
#include "RosBridge.h"
#include "BNO.h"
#include "Plot.h"

Movement *robot = nullptr;
bool ROS_ENABLE = true;
bool CHECK_PID = false;
bool CHECK_MOTORS = false;
bool CHECK_ENCODERS = true;

void setup() {
    Serial.begin(57600);
    Serial2.begin(57600);
    // if (ROS_ENABLE) {
    // }
    
    BNO bno;
    Movement initRobot;
    robot = &initRobot;
    robot->initEncoders();

    if (!ROS_ENABLE) {
        loop();
    }
    
    Plot plot(robot, ROS_ENABLE);
    // plot.startSequence();
    
    RosBridge bridge(robot, &bno, &plot);
    
    bridge.run();
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
                robot->cmdVelocity(0.5, 0, 0);
                plot.plotTargetandCurrent();
            }
        }
    }
    if (CHECK_ENCODERS) {
        Serial.begin(9600);
        bool direction = false;
        long long start_time = millis();
        while(1) {
          if (millis() - start_time > 5000) {
            start_time = millis();
            direction = !direction;
          }
          if (direction) {
            robot->right_motor_.forward();
            robot->right_motor_.changePwm(120);
            robot->left_motor_.forward();
            robot->left_motor_.changePwm(120);
          } else {
            robot->left_motor_.backward();
            robot->left_motor_.changePwm(120);
            robot->right_motor_.backward();
            robot->right_motor_.changePwm(120);
          }
               
          //Serial.print("R");
          //Serial.println( (robot->right_motor_.getPidTicks()));
            //   Serial.print("RD");
            //   Serial.println( (robot->right_motor_.getEncodersDir()));
          //Serial.print("L");
          //Serial.println( (robot->right_motor_.getPidTicks()));
          Serial.print("D ");
          Serial.print( (robot->right_motor_.getEncodersDir()));
          Serial.print("\t L ");
          Serial.println( (robot->left_motor_.getEncodersDir()));
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
