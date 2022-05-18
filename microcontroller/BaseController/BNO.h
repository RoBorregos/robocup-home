// This class has the declaration, initialization and usage function of the BNO.
#ifndef BNO_h
#define BNO_h

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNOADDR 0x28

class BNO {

  public:
    //////////////////////////////////Constructor//////////////////////////////////////
    BNO();


    //////////////////////////////////Calibration//////////////////////////////////////
    // Function that return a value between '0' (uncalibrated data) and '3' (fully calibrated).
    uint8_t orientationStatus();
    /////ADDITION/////
    void init();
    
    void updateEvents();
    float getQuat_x();
    float getQuat_y();
    float getQuat_z();
    float getQuat_w();
    float getAngVel_x();
    float getAngVel_y();
    float getAngVel_z();
    float getLinAcc_x();
    float getLinAcc_y();
    float getLinAcc_z();
    void anglesInfo();

    //////////////////////////////////Get Functions//////////////////////////////////////
    // Returns the angle in X axis considering bno_set_point_ and it is in a range of 0-360.
    int getCurrentXAngle();
    
    // Returns the angle in X axis.
    double getAngleX();
    
    // Returns the angle in Y axis.
    double getAngleY();
    
    // Returns the angle in Z axis.
    double getAngleZ();

    
  private:
    Adafruit_BNO055 bno_ = Adafruit_BNO055(55, 0x28); 
    sensors_event_t angVelocityData_, linearAccelData_;
    imu::Quaternion quat_;
    int bno_set_point_ = 0;
};

#endif
