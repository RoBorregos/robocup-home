Utils::Utils(){
}

int Utils::AngleToDirection(int angle){
    int diff=1000;
    for(int i=0;i<=8;i++){
        if(diff > abs(angle-i*45) ){
            diff=abs(angle-i*45);
        }else{
            return (i-1)*45;
        }
    }
    return 0;
}

void Utils::sendToPC_velocity(){
    if(millis()-this->timeMessage>35)
        return;

    byte* byteData1 = (byte*)(&moveAll.B_left.velocity);
    byte* byteData2 = (byte*)(&moveAll.F_left.velocity);
    byte* byteData3 = (byte*)(&moveAll.F_right.velocity);
    byte* byteData4 = (byte*)(&moveAll.B_right.velocity);
    byte* byteData5 = (byte*)(-1);
    byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    this->timeMessage=millis();
}

void Utils::sendToPC_ticks(){
    if(millis()-this->timeMessage>35)
        return;

    byte* byteData1 = (byte*)(&moveAll.B_left.lastticks);
    byte* byteData2 = (byte*)(&moveAll.F_left.lastticks);
    byte* byteData3 = (byte*)(&moveAll.F_right.lastticks);
    byte* byteData4 = (byte*)(&moveAll.B_right.lastticks);
    byte* byteData5 = (byte*)(-1);
    byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                    byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                    byteData3[0], byteData3[1], byteData3[2], byteData3[3],
                    byteData4[0], byteData4[1], byteData4[2], byteData4[3],
                    byteData5[0], byteData5[1], byteData5[2], byteData5[3]};
    Serial.write(buf, 20);

    this->timeMessage=millis();
}

void Utils::sendToPC_ticksT(){
    if(millis()-this->timeMessage>35)
        return;
        
    double tmp=moveAll.B_right.getTargetSpeed();
    double tmp1=moveAll.B_right.speedActual;
    double tmp2=moveAll.F_right.getTargetSpeed();
    double tmp3=moveAll.F_right.speedActual;
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

    this->timeMessage=millis();
}