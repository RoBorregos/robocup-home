//////////////////////////////////Constructor//////////////////////////////////////
Plot::Plot(Movement *moveAll){
    moveAll_ = moveAll;
    time_msg_ = millis();
}

//////////////////////////////////Plot Functions//////////////////////////////////////
void Plot::PlotMotorSpeed(){
    if(millis()-time_msg_>35){
        return;
    }
    int back_left_motor_current_speed = moveAll_->back_left_motor_.getCurrentSpeed();
    int front_left_motor_current_speed = moveAll_->front_left_motor_.getCurrentSpeed();
    int front_right_motor_current_speed = moveAll_->front_right_motor_.getCurrentSpeed();
    int back_right_motor_current_speed = moveAll_->back_right_motor_.getCurrentSpeed();
    byte* byteData1 = (byte*)(&back_left_motor_current_speed);
    byte* byteData2 = (byte*)(&front_left_motor_current_speed);
    byte* byteData3 = (byte*)(&front_right_motor_current_speed);
    byte* byteData4 = (byte*)(&back_right_motor_current_speed);
    byte* byteData5 = (byte*)(-1);
    byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    time_msg_=millis();
}

void Plot::PlotMotorTicks(){
    if(millis()-time_msg_>35){
        return;
    }
    int back_left_motor_last_ticks = moveAll_->back_left_motor_.getLastTicks();
    int front_left_motor_last_ticks = moveAll_->front_left_motor_.getLastTicks();
    int front_right_motor_last_ticks = moveAll_->front_right_motor_.getLastTicks();
    int back_right_motor_last_ticks = moveAll_->back_right_motor_.getLastTicks();
    byte* byteData1 = (byte*)(&back_left_motor_last_ticks);
    byte* byteData2 = (byte*)(&front_left_motor_last_ticks);
    byte* byteData3 = (byte*)(&front_right_motor_last_ticks);
    byte* byteData4 = (byte*)(&back_right_motor_last_ticks);
    byte* byteData5 = (byte*)(-1);
    byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    time_msg_=millis();
}

void Plot::PlotTargetandCurrent(){
    if(millis()-time_msg_>35){
        return;
    }
    double tmp = moveAll_->back_right_motor_.getTargetRpm(moveAll_->getTargetLinearVelocity());
    double tmp1 = moveAll_->back_right_motor_.getCurrentSpeed();
    double tmp2 = moveAll_->front_right_motor_.getTargetRpm(moveAll_->getTargetLinearVelocity());
    double tmp3 = moveAll_->front_right_motor_.getCurrentSpeed();
    byte* byteData1 = (byte*)(&tmp1);
    byte* byteData2 = (byte*)(&tmp);
    byte* byteData3 = (byte*)(&tmp2);
    byte* byteData4 = (byte*)(&tmp3);
    byte* byteData5 = (byte*)(-1);
    byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    time_msg_=millis();
}