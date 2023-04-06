#include "Odometry.h"

//////////////////////////////////Constructor//////////////////////////////////////
Odometry::Odometry(Movement *move_all, ros::NodeHandle *nh, Plot *plot) : move_all_(move_all), nh_(nh), plot_(plot),
velocity_subscriber_("/cmd_vel",&Odometry::velocityCallback, this),
encoder_publisher_("/base_control/encoders", &encoders_msg_){

    //Node Handle
    nh_->subscribe(velocity_subscriber_);
    nh_->advertise(encoder_publisher_);
    nh_->negotiateTopics();
    
    //Timers
    odom_timer_ = millis();
    watchdog_timer_ = millis();
    
    //Message Init
    encoders_msg_.encoders.time_delta = 0;
    
    encoders_msg_.encoders.left_wheel = 0;
    encoders_msg_.encoders.right_wheel = 0;

    for(int i = 0; i < kCountMotors; ++i) {
		last_encoder_counts_[i] = 0;
    }
}

//////////////////////////////////Velocity Suscriber//////////////////////////////////////
void Odometry::velocityCallback(const geometry_msgs::Twist& cmd_velocity) {
    linearX_ = cmd_velocity.linear.x;
    linearY_ = cmd_velocity.linear.y;
    angularZ_ = cmd_velocity.angular.z;
    watchdog_timer_ = millis();
}

//////////////////////////////////Encoders Publisher//////////////////////////////////////
void Odometry::getEncoderCounts() {
	int new_encoder_counts[kCountMotors];
	new_encoder_counts[0] = move_all_->left_motor_.getOdomTicks();
	new_encoder_counts[1] = move_all_->right_motor_.getOdomTicks();
	
	// find deltas
	int delta_encoder_counts[kCountMotors];
	for(int i = 0; i < kCountMotors; ++i) {
		// check for overflow
		if(abs(last_encoder_counts_[i]) > kCountOverflow && 
           abs(new_encoder_counts[i]) > kCountOverflow && 
           sign(last_encoder_counts_[i]) != sign(new_encoder_counts[i])) {

			if(sign(last_encoder_counts_[i]) > 0) {
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] + kIntMax;
            } else {
				delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i] - kIntMax;
            }

		} else {
            delta_encoder_counts[i] = new_encoder_counts[i] - last_encoder_counts_[i];
        } 
		
		if(abs(delta_encoder_counts[i]) > kCountReset) {
            delta_encoder_counts[i] = 0;
        } 
		
		last_encoder_counts_[i] = new_encoder_counts[i];
	}
    encoders_msg_.encoders.left_wheel = delta_encoder_counts[0];
    encoders_msg_.encoders.right_wheel = delta_encoder_counts[1];
}

void Odometry::publish() {
    if((millis() - odom_timer_) > kOdomPeriod) {
        getEncoderCounts();
        unsigned long currentTime = millis();
        encoders_msg_.encoders.time_delta = static_cast<float>(currentTime - odom_timer_) / 1000;
        
        // publish data
        encoder_publisher_.publish(&encoders_msg_);
        if((currentTime - kOdomPeriod) > (odom_timer_ + kOdomPeriod)) {
            odom_timer_ = currentTime;
        }
        else {
            odom_timer_ = odom_timer_ + kOdomPeriod;
        }
    }
}

//////////////////////////////////Run//////////////////////////////////////
void Odometry::run() {
    while(1) { 
        if((millis() - watchdog_timer_) > kWatchdogPeriod) {
            linearX_ = 0.0;
            linearY_ = 0.0;
            angularZ_ = 0.0;        
            watchdog_timer_ = millis();
        }
        move_all_->cmdVelocity(linearX_, linearY_, angularZ_, true);
        publish();
        plot_->plotTargetandCurrent();
        nh_->spinOnce();
    }
}
