#ifndef Plot_h
#define Plot_h

class Plot {
  public:
  
    Plot(Movement *moveAll);
    
    void plotMotorSpeed();
    void plotMotorTicks();
    void PlotTargetandActual();
    
    unsigned long timeMessage;
    
  private:
    Movement *moveAll_;


};

#endif
