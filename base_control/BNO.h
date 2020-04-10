#ifndef BNO_H
#define BNO_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int BNOSetPoint = 0;

class BNO {

  public:
    BNO();

    int actualAngle();
    double getAngleX();
    double getAngleY();
    double getAngleZ();
    
    void BNOCalibration();

    uint8_t orientationStatus();

    private:
      Adafruit_BNO055 bno_;
};
#endif