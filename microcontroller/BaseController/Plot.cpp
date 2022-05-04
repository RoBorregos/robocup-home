#include "Plot.h"
//////////////////////////////////Constructor//////////////////////////////////////
Plot::Plot(Movement *moveAll) {
    moveAll_ = moveAll;
    time_msg_ = millis();
}

//////////////////////////////////Plot Functions//////////////////////////////////////
void Plot::PlotMotorSpeed() {
    PlotData(
        moveAll_->back_left_motor_.getCurrentSpeed(),
        moveAll_->front_left_motor_.getCurrentSpeed(),
        moveAll_->front_right_motor_.getCurrentSpeed(),
        moveAll_->back_right_motor_.getCurrentSpeed()
    );
    /*BRSerial.print(moveAll_->back_left_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->front_left_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->front_right_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->back_right_motor_.getCurrentSpeed());
    Serial.println();*/
}

void Plot::PlotTargetandCurrent() {
    PlotData(
        moveAll_->back_right_motor_.getTargetRps(moveAll_->back_right_motor_.getTargetSpeed()),
        moveAll_->back_right_motor_.getCurrentSpeed(),
        moveAll_->front_right_motor_.getTargetRps(moveAll_->front_right_motor_.getTargetSpeed()),
        moveAll_->front_right_motor_.getCurrentSpeed()
    );
    /*Serial.print(moveAll_->back_right_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->back_right_motor_.getTargetRps(moveAll_->back_right_motor_.getTargetSpeed()));
    Serial.print(",");
    Serial.print(moveAll_->front_right_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->front_right_motor_.getTargetRps(moveAll_->front_right_motor_.getTargetSpeed()));
    Serial.print(",");
    Serial.print(moveAll_->back_left_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->back_left_motor_.getTargetRps(moveAll_->back_left_motor_.getTargetSpeed()));
    Serial.print(",");
    Serial.print(moveAll_->front_left_motor_.getCurrentSpeed());
    Serial.print(",");
    Serial.print(moveAll_->front_left_motor_.getTargetRps(moveAll_->front_left_motor_.getTargetSpeed()));
    Serial.println();*/
}
/*
void Plot::PlotData(const double data1, const double data2, const double data3, const double data4) {
     if(millis()-time_msg_>35) {
        return;
    }

    const byte* byteData1 = (byte*)(&data1);
    const byte* byteData2 = (byte*)(&data2);
    const byte* byteData3 = (byte*)(&data3);
    const byte* byteData4 = (byte*)(&data4);
    const byte* byteData5 = (byte*)(-1);
    
    const byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                          byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    time_msg_=millis();
}*/
void Plot::PlotData(const double data1, const double data2, const double data3, const double data4) {
     if(millis()-time_msg_>35) {
        return;
    }
    /*Serial.println("---------------/");
    Serial.println(data1);
    Serial.print(",");
    Serial.println(data2);
    Serial.print(",");
    Serial.println(data3);
    Serial.print(",");
    Serial.println(data4);
    Serial.println("---------------/");*/
    const byte* byteData1 = (byte*)(&data1);
    const byte* byteData2 = (byte*)(&data2);
    const byte* byteData3 = (byte*)(&data3);
    const byte* byteData4 = (byte*)(&data4);
    /*Serial.println("---------------/");
    Serial.print(byteData1[0]);
    Serial.print(",");
    Serial.print(byteData1[1]);
    Serial.print(",");
    Serial.print(byteData1[2]);
    Serial.print(",");
    Serial.println(byteData1[3]);
    Serial.println("---------------/");
    Serial.print(byteData2[0]);
    Serial.print(",");
    Serial.print(byteData2[1]);
    Serial.print(",");
    Serial.print(byteData2[2]);
    Serial.print(",");
    Serial.println(byteData2[3]);*/
    
    const byte buf[16] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                          byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                          byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                          byteData4[0], byteData4[1], byteData4[2], byteData4[3]};
    Serial.write(buf, 16);

    time_msg_=millis();
}
