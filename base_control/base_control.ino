#include "PID.h"
#include "BNO.h"
#include "Motor.h"
#include "Movement.h"
#include "Encoder.h"
#include "Odometry.h"
#include "Plot.h"
#include <math.h>

Movement *robot = NULL;

void setup(){
    Serial.begin(9600);
    while (!Serial){
        delay(1);
    } 

    BNO bno;
    
    Movement initRobot(&bno);
    robot = &initRobot;
    robot->initEncoders();

    Odometry odom(robot);
    Plot plot(robot);

    odom.run();
}

void loop(){   
}