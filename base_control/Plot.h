///This class prints data by serial communication in order to be displayed in graphs using a python application. This helps in order to see the PID controller development.
#ifndef Plot
#define Plot

class Plot {
  public:

    //////////////////////////////////Constructor//////////////////////////////////////
    Plot(Movement *moveAll);
    
    //////////////////////////////////Plot Functions//////////////////////////////////////
    //This function print the actual speed of the four motors.
    void PlotMotorSpeed();
    //This function print the actual ticks of the four motors.
    void PlotMotorTicks();
    //This function print the actual and the target speed.
    void PlotTargetandActual();
    
    
  private:
    Movement *moveAll_;
    unsigned long time_msg_;


};

#endif
