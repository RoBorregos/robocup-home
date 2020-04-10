Odometry::Odometry() : vel_sub("cmd_vel",&Odometry::vel_callback,this), enc_pub("encoders", &enc_msg) {
    //Node Handle
    nh.initNode();
    nh.subscribe(vel_sub);
    nh.advertise(enc_pub);
    
    while(!nh.connected()) 
        nh.spinOnce();
    
    //Timers
    odom_timer = millis();
    watchdog_timer = millis();
    
    //Message Init
    enc_msg.data = (float *)malloc(sizeof(float)*4);
    enc_msg.data_length = 4;

    for(int i = 0; i < 4; i++)
		this->lastEncoderCounts[i] = 0;
}

void Odometry::vel_callback(const geometry_msgs::Twist& cmdvel) {
    cmd_vel(
        min(cmdvel.linear.x, LINEAR_X_MAX_VEL),
        min(cmdvel.linear.y, LINEAR_Y_MAX_VEL),
        min(cmdvel.angular.z, ANGULAR_Z_MAX_VEL)
    );
    watchdog_timer = millis();
}
void Odometry::cmd_vel(double linearx,double lineary, double angularz){
    if(angularz > linearx && angularz > lineary){
        moveAll.dteta=angularz;
        moveAll.pidAngularMovement();
    }else{
        moveAll.dX=linearx;
        moveAll.dY=lineary;
        moveAll.pidLinearMovement();
    }
}

void Odometry::getEncoderCounts(){
	int newEncoderCounts[4];
	newEncoderCounts[0] = moveAll.F_left.getOdomTicks();
	newEncoderCounts[1] = moveAll.F_right.getOdomTicks();
	newEncoderCounts[2] = moveAll.B_left.getOdomTicks();
	newEncoderCounts[3] = moveAll.B_right.getOdomTicks();
	
	// find deltas
	int deltaEncoderCounts[4];
	for(int i = 0; i < 4; i++) {
		// check for overflow
		if(abs(this->lastEncoderCounts[i]) > COUNT_OVERFLOW && abs(newEncoderCounts[i]) > COUNT_OVERFLOW && sign(this->lastEncoderCounts[i]) != sign(newEncoderCounts[i])) {
			if(sign(this->lastEncoderCounts[i]) > 0)
				deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i] + INT_MAX;
			else
				deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i] - INT_MAX;
		}
		else deltaEncoderCounts[i] = newEncoderCounts[i] - this->lastEncoderCounts[i];
		
		if(abs(deltaEncoderCounts[i]) > COUNT_RESET) deltaEncoderCounts[i] = 0;
		
		this->lastEncoderCounts[i] = newEncoderCounts[i];
	}

	enc_msg.data[0] = (deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;
	enc_msg.data[1] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] + deltaEncoderCounts[2] - deltaEncoderCounts[3]) / 4;
	enc_msg.data[2] = (0 - deltaEncoderCounts[0] + deltaEncoderCounts[1] - deltaEncoderCounts[2] + deltaEncoderCounts[3]) / 4;

}

void Odometry::publish(){
    if((millis() - odom_timer) > ODOM_PERIOD) {
        getEncoderCounts();
        unsigned long currentTime = millis();
        enc_msg.data[3] = (float)(currentTime - odom_timer) / 1000;
        
        // publish data
        enc_pub.publish(&enc_msg);
        if((currentTime - ODOM_PERIOD) > (odom_timer + ODOM_PERIOD)) {
            odom_timer = currentTime;
        }
        else {
            odom_timer = odom_timer + ODOM_PERIOD;
        }
    }
}