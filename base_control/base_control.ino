
//PID
#include "PID_v.h"


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
    
}
