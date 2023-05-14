#ifndef Plot_h
#define Plot_h

#include <Arduino.h>
#include "Movement.h"

// This class prints data by serial communication in order to be displayed in graphs using a
// python application. This helps in order to see the PID controller development.
class Plot
{
private:
  // Prints data sent in parameters.
  void plotData(const double data1, const double data2, const double data3, const double data4, const double data5);
  
  
  Movement *moveAll_;
  bool useSerial2_;
  unsigned long timeMsg;

public:
  // Constructor
  Plot(Movement *moveAll, bool useSerial2);

  // Plot Functions

  // This function print the current speed of the four motors.
  void plotMotorSpeed();

  // This function print the current ticks of the four motors.
  void plotMotorTicks();

  // This function print the current and the target speed.
  void plotTargetandCurrent();

  void plotPWM();

  void plotTest();

  // Sends start sequence to signal the beginning of plot input. Call only once, and right before 
  // calling the plotting functions for the first time.
  void startSequence();
};

#endif
