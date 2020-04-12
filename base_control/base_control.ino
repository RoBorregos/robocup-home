#include "PID.h"
#include "BNO.h"
#include "Motor.h"
#include "Movement.h"
#include "Encoder.h"
#include "Odometry.h"
#include "Plot.h"
#include <math.h>

Movement *robot = nullptr;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(1);
    } 
    Serial.println("Serial Initialization completed.");

    BNO bno;
    Serial.println("Bno Initialization completed.");

    Movement initRobot(&bno);
    robot = &initRobot;
    robot->initEncoders();
    Serial.println("Movement Initialization completed.");

    Odometry odom(robot);
    Serial.println("Odometry Initialization completed.");

    Plot plot(robot);
    Serial.println("Plot Initialization completed.");


    odom.run();
}

void loop() {   
}