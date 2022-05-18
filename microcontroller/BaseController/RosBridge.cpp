#include "RosBridge.h"

//////////////////////////////////Constructor//////////////////////////////////////
RosBridge::RosBridge(Movement *move_all, BNO *bno, ros::NodeHandle *nh) : move_all_(move_all), bno_(bno), nh_(nh), 
velocity_subscriber_("/cmd_vel",&RosBridge::cmdVelocityCallback, this),
front_encoder_publisher_("/base_control/front/encoders", &front_encoders_msg_),
back_encoder_publisher_("/base_control/back/encoders", &back_encoders_msg_),
bno_sensor_publisher_odom_("/sensor/imu", &bno_sensor_msgs_odom_){

    //Node Handle
    nh_->subscribe(velocity_subscriber_);
    nh_->advertise(front_encoder_publisher_);
    nh_->advertise(back_encoder_publisher_);
    nh_->advertise(bno_sensor_publisher_odom_);
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

      //BNO odom msg init
 
    char Bno[] = "/imu0";
    bno_sensor_msgs_odom_.header.frame_id = Bno;
    bno_sensor_msgs_odom_.header.stamp = nh_->now(); 

    bno_sensor_msgs_odom_.angular_velocity_covariance[0] = -1;
    bno_sensor_msgs_odom_.linear_acceleration_covariance[0] = -1;
    bno_sensor_msgs_odom_.orientation_covariance[0] = -1;

    bno_sensor_msgs_odom_.orientation.x = 0.0;
    bno_sensor_msgs_odom_.orientation.y = 1.0;
    bno_sensor_msgs_odom_.orientation.z = 0.0;
    bno_sensor_msgs_odom_.orientation.w = 0.0;
      
    bno_sensor_msgs_odom_.angular_velocity.x = 0.0;
    bno_sensor_msgs_odom_.angular_velocity.y = 0.0;
    bno_sensor_msgs_odom_.angular_velocity.z = 0.0;
    
    bno_sensor_msgs_odom_.linear_acceleration.x = 0.0;
    bno_sensor_msgs_odom_.linear_acceleration.y = 0.0;
    bno_sensor_msgs_odom_.linear_acceleration.z = 0.0;
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

        //BNO odom data
    
        //bno_sensor_msgs_odom_.header.stamp = nh_->now();
    
        bno_sensor_msgs_odom_.orientation.x = bno_->getQuat_x();
        bno_sensor_msgs_odom_.orientation.y = bno_->getQuat_y();
        bno_sensor_msgs_odom_.orientation.z = bno_->getQuat_z();
        bno_sensor_msgs_odom_.orientation.w = bno_->getQuat_w();
        bno_sensor_msgs_odom_.linear_acceleration.x = bno_->getLinAcc_x();
        bno_sensor_msgs_odom_.linear_acceleration.y = bno_->getLinAcc_y();
        bno_sensor_msgs_odom_.linear_acceleration.z = bno_->getLinAcc_z();
    
        bno_sensor_msgs_odom_.angular_velocity.x = bno_->getAngVel_x();
        bno_sensor_msgs_odom_.angular_velocity.y = bno_->getAngVel_y();
        bno_sensor_msgs_odom_.angular_velocity.z = bno_->getAngVel_z();

        unsigned long currentTime = millis();
        front_encoders_msg_.encoders.time_delta = static_cast<float>(currentTime - odom_timer_) / 1000;
        back_encoders_msg_.encoders.time_delta = static_cast<float>(currentTime - odom_timer_) / 1000;
        
        // publish data
        front_encoder_publisher_.publish(&front_encoders_msg_);
        back_encoder_publisher_.publish(&back_encoders_msg_);
        bno_sensor_publisher_odom_.publish(&bno_sensor_msgs_odom_);
        
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
