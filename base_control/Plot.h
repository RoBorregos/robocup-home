// This class prints data by serial communication in order to be displayed in graphs using a 
// python application. This helps in order to see the PID controller development.
#ifndef Plot_h
#define Plot_h

class Plot {
  public:

    //////////////////////////////////Constructor//////////////////////////////////////
    Plot(Movement *moveAll);
    
    //////////////////////////////////Plot Functions//////////////////////////////////////
    // This function print the current speed of the four motors.
    void PlotMotorSpeed();
    
    // This function print the current ticks of the four motors.
    void PlotMotorTicks();
    
    // This function print the current and the target speed.
    void PlotTargetandCurrent();
    
    
  private:
    Movement *moveAll_;
    unsigned long time_msg_;


};

#endif
