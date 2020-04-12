///This class has the declaration,initialization and usage function of the BNO.
#ifndef BNO
#define BNO

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class BNO {

  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    BNO();


    //////////////////////////////////Calibration//////////////////////////////////////
    ///Function that return a value between '0' (uncalibrated data) and '3' (fully calibrated).
    uint8_t orientationStatus();


    //////////////////////////////////Get Functions//////////////////////////////////////
    int getActualAngle();
    double getAngleX();
    double getAngleY();
    double getAngleZ();

    
  private:
    Adafruit_BNO055 bno_;
    int BNOSetPoint = 0;
};
#endif