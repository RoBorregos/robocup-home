// This class has all the functions related to the ROS connection. It receives the velocity
// commands and publish the encoders ticks data for Odometry.
// TODO: (@josecisneros001) Publish Imu Data
#ifndef RosBridge_h
#define RosBridge_h

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <base_control/StampedEncoders.h>
#include <base_control/Encoders.h>

#include <stdint.h>
#include <math.h>
#include <Arduino.h>

#include "BNO.h"
#include "Movement.h"


class RosBridge{
    public:
        //////////////////////////////////Constructor//////////////////////////////////////
        RosBridge(Movement *move_all, BNO *bno, ros::NodeHandle *nh);
        
        
        //////////////////////////////////Run//////////////////////////////////////
        // Calls watchdog and publish.
        void run();

    private:
        //////////////////////////////////Velocity Suscriber//////////////////////////////////////
        // Receives velocity commands.
        void cmdVelocityCallback(const geometry_msgs::Twist& cmdvel);

        // Verify it is still receiving velocity commands.
        void watchdog();
        
        //////////////////////////////////Encoders Publisher//////////////////////////////////////
        // Publish encoder message.
        void publish();
        
        Movement *move_all_;

        BNO *bno_;
        
        // Node.
        ros::NodeHandle  *nh_;
        
        // Suscriber.
        ros::Subscriber<geometry_msgs::Twist, RosBridge> velocity_subscriber_;
        static constexpr uint16_t kWatchdogPeriod = 500;
        
        // Publisher.
        ros::Publisher front_encoder_publisher_;
        ros::Publisher back_encoder_publisher_;

         //ODOM BNO PUBLISHER
        ros::Publisher bno_sensor_publisher_odom_;
        
        base_control::StampedEncoders front_encoders_msg_;
        base_control::StampedEncoders back_encoders_msg_;

        // ODOM BNO MSG
        sensor_msgs::Imu bno_sensor_msgs_odom_;
        
        static constexpr uint8_t kOdomPeriod = 40;

        // Timers.
        unsigned long odom_timer_ = 0;
        unsigned long watchdog_timer_ = 0;
        
};

#endif
