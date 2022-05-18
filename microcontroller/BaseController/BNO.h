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
    BNO(ros::NodeHandle *nh);

    //////////////////////////////////Calibration//////////////////////////////////////
    // Function that return a value between '0' (uncalibrated data) and '3' (fully calibrated).
    uint8_t orientationStatus();
    
    void updateEvents();
    void getImuInfo();

    //////////////////////////////////Get Functions//////////////////////////////////////
    // Returns the angle in X axis.
    double getAngleX();
    
    // Returns the angle in Y axis.
    double getAngleY();
    
    // Returns the angle in Z axis.
    double getAngleZ();

    
  private:
    // ROS
    ros::NodeHandle * nh_;
    
    Adafruit_BNO055 bno_ = Adafruit_BNO055(55, 0x28); 
    sensors_event_t angVelocityData_, linearAccelData_;
    imu::Quaternion quat_;
};

#endif
