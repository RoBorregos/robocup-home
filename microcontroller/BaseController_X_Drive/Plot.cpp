#include "Plot.h"

// Constructor

Plot::Plot(Movement *moveAll, bool useSerial2)
{
    moveAll_ = moveAll;
    useSerial2_ = useSerial2;
    timeMsg = millis();
}

// Plot Functions

void Plot::plotMotorSpeed()
{
    plotData(
        moveAll_->back_left_motor_.getCurrentSpeed(),
        moveAll_->front_left_motor_.getCurrentSpeed(),
        moveAll_->front_right_motor_.getCurrentSpeed(),
        moveAll_->back_right_motor_.getCurrentSpeed(),
        0.0
    );
}

void Plot::plotTest()
{
    plotData(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    );
}


void Plot::plotTargetandCurrent()
{/*
    plotData(
        abs(moveAll_->back_right_motor_.getTargetRps(moveAll_->back_right_motor_.getTargetSpeed())),
        moveAll_->back_right_motor_.getCurrentSpeed(),
        abs(moveAll_->front_right_motor_.getTargetRps(moveAll_->front_right_motor_.getTargetSpeed())),
        moveAll_->front_right_motor_.getCurrentSpeed(),
        0.0
    );
    */
}

void Plot::plotPWM()
{
    plotData(
        moveAll_->back_left_motor_.getPWM(),
        moveAll_->front_left_motor_.getPWM(),
        moveAll_->front_right_motor_.getPWM(),
        moveAll_->back_right_motor_.getPWM(),
        0.0
    );
}


void Plot::plotData(const double data1, const double data2, const double data3, const double data4, const double data5)
{
    if (millis() - timeMsg < 35)
    {
        return;
    }
    const byte *byteData1 = (byte *)(&data1);
    const byte *byteData2 = (byte *)(&data2);
    const byte *byteData3 = (byte *)(&data3);
    const byte *byteData4 = (byte *)(&data4);
    const byte *byteData5 = (byte *)(&data5);

    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    if (useSerial2_) {
      Serial2.write(buf, 20);
    } else {
      Serial.write(buf, 20);
    }
    
    timeMsg = millis();
}

void Plot::startSequence(){
    if (useSerial2_) {
      Serial2.write("<target>"); // Sequence to recognize at "multiplePlots.py"
    } else {
      Serial.write("<target>"); // Sequence to recognize at "multiplePlots.py"
    }
}
