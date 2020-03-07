//General
#include<math.h>
#define PI_C 3.14159265358979323846
unsigned long time=0;

//PID
#include "PID.h"


//BNO
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int BNOSetPoint = 0;



//Motor
#define TIME_VELOCITY_SAMPLE 100.0 //millis
#define PULSES_PER_REVOLUTION 4320.0
#define WHEEL_DIAMETER 0.1 //meters
#define MINPWM 100
#define MAXTICKS 286
#define MINTICKS 190

#include "Motor.h"

//Movement
#include "Movement.h"
Movement moveAll;



//PID TEST
double pv_speed=0;
double e_speed=0;
double e_speed_sum=0;
double e_speed_pre=0;
double set_speed=0;
double pwm_pulse=0;
double kp=60;
double ki=55;
double kd=45;

//Utils
#include "Utils.h"
Utils util;

void setup() {
    
    //Initialize Serial
    Serial.begin(38400);
    while (! Serial) {
        delay(1);
    }
 
    //Initialize BNO
    bno.begin();
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    bno.getEvent(&event);
    BNOSetPoint = event.orientation.x;
    
    time=millis();
    util.timeMessage=millis();
}


void loop() {
    
    moveAll.B_right.Forward();
    moveAll.F_right.Forward();
    moveAll.B_left.Forward();
    moveAll.F_left.Forward();
    
    
    moveAll.B_right.constantSpeed();
    moveAll.F_right.constantSpeed();
    moveAll.B_left.constantSpeed();
    moveAll.F_left.constantSpeed();
    
    util.sendToPC_ticksT();
}
