#include "RosBridge.h"

//////////////////////////////////Constructor//////////////////////////////////////
RosBridge::RosBridge(Movement *move_all, ros::NodeHandle *nh) : move_all_(move_all), nh_(nh), 
velocity_subscriber_("/base_control/cmd_vel",&RosBridge::cmdVelocityCallback, this),
front_encoder_publisher_("/base_control/front/encoders", &front_encoders_msg_),
back_encoder_publisher_("/base_control/back/encoders", &back_encoders_msg_) {

    //Node Handle
    nh_->subscribe(velocity_subscriber_);
    nh_->advertise(front_encoder_publisher_);
    nh_->advertise(back_encoder_publisher_);
    nh_->negotiateTopics();
    
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
}

//////////////////////////////////Velocity Suscriber//////////////////////////////////////
void RosBridge::cmdVelocityCallback(const geometry_msgs::Twist& cmd_velocity) {
    move_all_->cmdVelocity(
        cmd_velocity.linear.x, 
        cmd_velocity.linear.y,
        cmd_velocity.angular.z
    );
    watchdog_timer_ = millis();
}

void RosBridge::watchdog() {
    if((millis() - watchdog_timer_) > kWatchdogPeriod) {
        move_all_->stop();
        watchdog_timer_ = millis();
    }
}

void RosBridge::publish() {
    if((millis() - odom_timer_) > kOdomPeriod) {
        int delta_encoder_counts[4];
        move_all_->getEncoderCounts(delta_encoder_counts);
        front_encoders_msg_.encoders.left_wheel = delta_encoder_counts[0];
        front_encoders_msg_.encoders.right_wheel = delta_encoder_counts[1];
        back_encoders_msg_.encoders.left_wheel = delta_encoder_counts[2];
        back_encoders_msg_.encoders.right_wheel = delta_encoder_counts[3];

        unsigned long currentTime = millis();
        front_encoders_msg_.encoders.time_delta = static_cast<float>(currentTime - odom_timer_) / 1000;
        back_encoders_msg_.encoders.time_delta = static_cast<float>(currentTime - odom_timer_) / 1000;
        
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
void RosBridge::run() {
    while(1) {
        watchdog();
        publish();
        nh_->spinOnce();
    }
}
