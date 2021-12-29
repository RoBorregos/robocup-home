#include "RosBridge.h"

//////////////////////////////////Constructor//////////////////////////////////////
RosBridge::RosBridge(Movement *move_all, ros::NodeHandle *nh) : move_all_(move_all), nh_(nh), 
velocity_subscriber_("/base_control/cmd_vel",&RosBridge::cmdVelocityCallback, this),
encoders_publisher_("/base_control/encoders", &enc_msg_) {

    //Node Handle
    nh_->subscribe(velocity_subscriber_);
    nh_->advertise(encoders_publisher_);
    nh_->negotiateTopics();
    
    //Timers
    odom_timer_ = millis();
    watchdog_timer_ = millis();
    
    //Message Init
    enc_msg_.data = (float *)malloc(sizeof(float)*4);
    enc_msg_.data_length = 4;
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
        move_all_->getEncoderCounts(enc_msg_.data);
        unsigned long currentTime = millis();
        enc_msg_.data[3] = static_cast<float>(currentTime - odom_timer_) / 1000;
        
        // publish data
        encoders_publisher_.publish(&enc_msg_);

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
