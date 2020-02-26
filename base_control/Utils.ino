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