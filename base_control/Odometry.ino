//////////////////////////////////Constructor//////////////////////////////////////
Odometry::Odometry(Movement *move_all) : velocity_subscriber_("/base_control/cmd/velocity",&Odometry::velocityCallback, this),  front_encoder_publisher_("/base_control/front/encoders", &front_encoders_msg_), back_encoder_publisher_("/base_control/back/encoders", &back_encoders_msg_){
    move_all_ = move_all;
    //Node Handle
    nh_.initNode();
    nh_.subscribe(velocity_subscriber_);
    nh_.advertise(front_encoder_publisher_);
    nh_.advertise(back_encoder_publisher_);
    
    while(!nh_.connected()){
        nh_.spinOnce();
    }

    
    //Timers
    odom_timer_ = millis();
    watchdog_timer_ = millis();
    
    //Message Init
    front_encoders_msg_.encoders.time_delta = 0;
    back_encoders_msg_.encoders.time_delta = 0;
    
    front_encoders_msg_.encoders.left_wheel = 0;
    front_encoders_msg_.encoders.right_wheel = 0;
    back_encoders_msg_.encoders.left_wheel = 0;
    back_encoders_msg_.encoders.right_wheel = 0;

    for(int i = 0; i < kCountMotors; ++i){
		last_encoder_counts_[i] = 0;
    }
}

//////////////////////////////////Velocity Suscriber//////////////////////////////////////
void Odometry::velocityCallback(const geometry_msgs::Twist& cmd_velocity) {
    cmdVelocity(
        min(cmd_velocity.linear.x, kLinearXMaxVelocity), 
        min(cmd_velocity.linear.y, kLinearYMaxVelocity), 
        min(cmd_velocity.angular.z, kAngularZMaxVelocity)
    );
    watchdog_timer_ = millis();
}

void Odometry::cmdVelocity(const double linear_x, const double linear_y, const double angular_z){
    if(angular_z > linear_x && angular_z > linear_y){
        move_all_->setDeltaAngular(angular_z);
        move_all_->pidAngularMovement();
    }else{
        move_all_->setDeltaX(linear_x);
        move_all_->setDeltaY(linear_y);
        move_all_->pidLinearMovement();
    }
}

//////////////////////////////////Encoders Publisher//////////////////////////////////////
void Odometry::getEncoderCounts(){
	int new_encoder_counts[kCountMotors];
	new_encoder_counts[0] = move_all_->front_left_motor_.getOdomTicks();
	new_encoder_counts[1] = move_all_->front_right_motor_.getOdomTicks();
	new_encoder_counts[2] = move_all_->back_left_motor_.getOdomTicks();
	new_encoder_counts[3] = move_all_->back_right_motor_.getOdomTicks();
	
	// find deltas
	int delta_encoder_counts[kCountMotors];
	for(int i = 0; i < kCountMotors; ++i) {
		// check for overflow
		if(abs(last_encoder_counts_[i]) > kCountOverflow && abs(new_encoder_counts[i]) > kCountOverflow && sign(last_encoder_counts_[i]) != sign(new_encoder_counts[i])) {
			if(sign(last_encoder_counts_[i]) > 0){
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] + kIntMax;
            } else{
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] - kIntMax;
            }
		} else{
            delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i];
        } 
		
		if(abs(delta_encoder_counts[i]) > kCountReset) delta_encoder_counts[i] = 0;
		
		last_encoder_counts_[i] = new_encoder_counts[i];
	}
    front_encoders_msg_.encoders.left_wheel = delta_encoder_counts[0];
    front_encoders_msg_.encoders.right_wheel = delta_encoder_counts[1];
    back_encoders_msg_.encoders.left_wheel = delta_encoder_counts[2];
    back_encoders_msg_.encoders.right_wheel = delta_encoder_counts[3];
}

void Odometry::publish(){
    if((millis() - odom_timer_) > kOdomPeriod) {
        getEncoderCounts();
        unsigned long currentTime = millis();
        front_encoders_msg_.encoders.time_delta = (float)(currentTime - odom_timer_) / 1000;
        back_encoders_msg_.encoders.time_delta = (float)(currentTime - odom_timer_) / 1000;
        
        
        // publish data
        front_encoder_publisher_.publish(&front_encoders_msg_);
        back_encoder_publisher_.publish(&back_encoders_msg_);
        if((currentTime - kOdomPeriod) > (odom_timer_ + kOdomPeriod)) {
            odom_timer_ = currentTime;
        }
        else {
            odom_timer_ = odom_timer_ + kOdomPeriod;
        }
    }
}

//////////////////////////////////Run//////////////////////////////////////
void Odometry::run(){
    while(1) { 
        if((millis() - watchdog_timer_) > kWatchdogPeriod) {
            move_all_->stop();
            watchdog_timer_ = millis();
        }
        publish();
        nh_.spinOnce();
    }
}