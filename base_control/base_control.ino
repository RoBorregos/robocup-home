
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
#define TIME_VELOCITY_SAMPLE 250 //millis
#define PULSES_PER_REVOLUTION 4320
#define WHEEL_DIAMETER 0.1 //meters
#include "Motor.h"

//Movement
#include "Movement.h"
Movement moveAll;

//Utils
#include "Utils.h"
Utils util;


#define PI_C 3.14159265358979323846


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

    //Initial Time
    moveAll.VelocityTiming = millis();

}


void loop() {
    
    Serial.println();
    moveAll.pwm(255);
    moveAll.F_left.Forward();
    moveAll.F_right.Forward();
    moveAll.B_left.Forward();
    moveAll.B_right.Forward();
    delay(2000);
    moveAll.F_left.Backward();
    moveAll.F_right.Backward();
    moveAll.B_left.Backward();
    moveAll.B_right.Backward();
    delay(2000);
    
    
    moveAll.calcVelocity();
}
