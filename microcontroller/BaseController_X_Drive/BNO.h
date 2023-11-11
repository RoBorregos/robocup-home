// This class has the declaration, initialization and usage function of the BNO.
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
    // Function that return a value between '0' (uncalibrated data) and '3' (fully calibrated).
    uint8_t orientationStatus();

    void reset();


    //////////////////////////////////Get Functions//////////////////////////////////////
    void updateBNO();
    float getYaw();
    float getYawVel();
    float getXAccel();
    float getYAccel();
    float getZAccel();

    
  private:
    Adafruit_BNO055 bno_;
    int reset_pin_ = 22;

    float yaw_ = 0.0;
    float yaw_vel_ = 0.0;
    float x_accel = 0.0;
    float y_accel = 0.0;
    float z_accel = 0.0;
};
#endif
