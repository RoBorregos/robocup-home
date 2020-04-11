#ifndef Odometry_h
#define Odometry_h

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <base_control/StampedEncoders.h>
#include <base_control/Encoders.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>


#define ODOM_PERIOD 50 // 20 Hz
#define WATCHDOG_PERIOD 500 // 2 Hz
#define LINEAR_X_MAX_VEL 1.1 // m/s
#define LINEAR_Y_MAX_VEL 1.3 // m/s
#define ANGULAR_Z_MAX_VEL 2.8 // rad/s
#define DEADZONE 5 // in bytes around 128

#define INT_MAX 65535
#define COUNT_RESET 250
#define COUNT_OVERFLOW 16374
#define sign(a) (min(1, max(-1, a)))


class Odometry{
    public:
        Odometry(Movement *moveAll);
        void vel_callback(const geometry_msgs::Twist& cmdvel);
        void cmd_vel(double linearx,double lineary, double angularz);
        void getEncoderCounts();
        void publish();
        void run();
        unsigned long odom_timer = 0;
        unsigned long watchdog_timer = 0;
        ros::NodeHandle  nh;
    private:
        Movement *moveAll_;
        int lastEncoderCounts[4];
        std_msgs::Float32MultiArray enc_msg;
        ros::Publisher enc_pub;
        ros::Subscriber<geometry_msgs::Twist,Odometry> vel_sub;
        
};


#endif