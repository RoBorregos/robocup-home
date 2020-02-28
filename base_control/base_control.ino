
//PID
#include "PID_v3.h"


//BNO

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int BNOSetPoint = 0;

bool rampa = false;
int orientationStatus();


//Motor
#include "Motor.h"

//Movement
#include "Movement.h"
Movement moveAll;

//Utils
#include "Utils.h"
Utils util;


void setup() {
    
    //Initialize Serial
    Serial.begin(9600);
    while (! Serial) {
        delay(1);
    }
 
    //Initialize BNO
    bno.begin();
    bno.setExtCrystalUse(true);
    sensors_event_t event;
    bno.getEvent(&event);
    BNOSetPoint = event.orientation.x;


}


void loop() {
    moveAll.pwm(250);
    
    int time = millis();
    while(true)
        if(millis()-time < 5000){
            movePID(moveAll.F_left,moveAll.F_right);

        }else{
            moveAll._stop();
        }
}


void movePID(Motor& a,Motor& b){
    a.Forward();b.Forward();
    a._IPID(a.ticks,b.ticks);
    a._PID.Compute();
    b._IPID(b.ticks,a.ticks);
    b._PID.Compute();

    a.changePWM(200+a._OPID());
    b.changePWM(200+b._OPID());
    
    Serial.print("a:");
    Serial.println(200+a._OPID());
    Serial.print("b:");
    Serial.println(200+b._OPID());

    Serial.print("a - ticks :");
    Serial.println(a.ticks);
    Serial.print("b - ticks :");
    Serial.println(b.ticks);
}
