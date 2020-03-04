//General
#include<math.h>
#define PI_C 3.14159265358979323846
int time=0;

//PID
#include "PID_v.h"


//BNO
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int BNOSetPoint = 0;



//Motor
#define TIME_VELOCITY_SAMPLE 500.0 //millis
#define PULSES_PER_REVOLUTION 4320.0
#define WHEEL_DIAMETER 0.1 //meters
#define MINPWM 60
#include "Motor.h"

//Movement
#include "Movement.h"
Movement moveAll;

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
    moveAll._move90();
    
    movePID(moveAll.F_right);
    movePID(moveAll.F_left);
    movePID(moveAll.B_left);
    movePID(moveAll.B_right);

    moveAll.calcVelocity();
    util.sendToPC_ticks();
}


//PID 1.1
void movePID(Motor& a,Motor& b){
    a.Forward();b.Forward();
    a._IPID(a.lastticks,b.lastticks);
    a._PID.Compute();
    b._IPID(b.lastticks,a.lastticks);
    b._PID.Compute();

    a.changePWM(200+a._OPID());
    b.changePWM(200+b._OPID());
}

//PID 1.2
void movePID(Motor& a){
    a._IPID(a.lastticks,moveAll.getTargetTicks());
    a._PID.Compute();
    a.changePWM(MINPWM+a._OPID());
}

