///This class has the declaration,initialization and usage function of the BNO.
#ifndef BNO_h
#define BNO_h

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
    //Returns the angle considering bno_set_point_ and it is in a range of 0-360
    int getActualAngle();
    //Returns the angle in X axis 
    double getAngleX();
    //Returns the angle in Y axis
    double getAngleY();
    //Returns the angle in Z axis
    double getAngleZ();

    
  private:
    Adafruit_BNO055 bno_;
    int bno_set_point_ = 0;
};
#endif