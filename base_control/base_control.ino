//General
#include "Namespaces.h"
#include <math.h>
#define PI_C 3.14159265358979323846

//PID
#include "PID.h"

//BNO
#include "BNO.h"
BNO bno_;

//Motor
#include "Motor.h"

//Movement
#include "Movement.h"
Movement moveAll;

//ROS Odometry
#include "Odometry.h"
Odometry odom;

//Utils
#include "Utils.h"
Utils util;


void setup(){
    //Initialize Serial
    Serial.begin(9600);
    while (!Serial) delay(1);

    
    util.timeMessage = millis();
}

void loop(){   
    if((millis() - odom.watchdog_timer) > WATCHDOG_PERIOD) {
        moveAll._stop();
        odom.watchdog_timer = millis();
    }
    odom.publish();
    odom.nh.spinOnce();
}