// This script converts "encoders" messages into proper odom messages.

#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "base_control/StampedEncoders.h"
#include <geometry_msgs/Quaternion.h>


// Define Velocity struct for clarity in the code.
struct Velocity {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
};

// Define Position struct for clarity in the code.
struct Position {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
};

// Initialize global variables.
int left = 0;
int right = 0;
double dt = 0.000001;

// Initialize constants.
const double wheel_radius = 0.06;
const double wheel_circumference = wheel_radius * 2 * M_PI;
const double encoder_resolution = 600;
// The robot constant is defined by the sum of the distance between 
// the wheel's x-coord & the origin, and the y-coord & the origin.
const double robot_constant = 0.33;
const double dist_per_tick = wheel_circumference / encoder_resolution;
const bool publish_odom_message = true;
const bool publish_odom_frame = true;

// Function to save encoder data to global variables.
void halCallBack(const base_control::StampedEncoders::ConstPtr& msg) {
    ROS_INFO_STREAM("INFO RECEIVED \n *Encoders");
    left = msg->encoders.left_wheel;
    right = msg->encoders.right_wheel;
    dt = msg->encoders.time_delta;
}

// Function to save back encoder data to global variables.
void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

// Function to initalize Odometry Message.
void initializeOdometryMessage(nav_msgs::Odometry &odom){
    // Define frame constants.
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    // Define postiion constants.
    odom.pose.pose.position.z = 0.0;
    
    // Define velocity constants.
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    
    // Define pose covariance constants.
    odom.pose.covariance[0] = 1e3;
    odom.pose.covariance[7] = 1e3;
    odom.pose.covariance[14] = 1e100;
    odom.pose.covariance[21] = 1e100;
    odom.pose.covariance[28] = 1e100;
    odom.pose.covariance[35] = 1e3;

    // Define twist covariance constants.
    odom.twist.covariance[0] = 1e3;
    odom.twist.covariance[7] = 1e3;
    odom.twist.covariance[14] = 1e100;
    odom.twist.covariance[21] = 1e100;
    odom.twist.covariance[28] = 1e100;
    odom.twist.covariance[35] = 1e3;
}

// Function to initalize Odometry Transform.
void initializeOdometryTransform(geometry_msgs::TransformStamped &odom_trans){
    // Define frame constants.
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    
    // Define postiion constants.
    odom_trans.transform.translation.z = 0.0;
}

// Function to compute robot velocity.
void computeRobotVelocity(Velocity &robot_velocity){
    // Compute the velocities of each wheel.
    const double v_l = (left * dist_per_tick) / dt;
    const double v_r = (right * dist_per_tick) / dt;

    // Compute the overall velocity of the robot.
    robot_velocity.x = (v_l + v_r) / 2.0;
    robot_velocity.th = (-v_l + v_r) / (2.0 * robot_constant);
}

// Function to compute robot displacement.
void computeRobotDisplacement(const Velocity robot_velocity, Position &robot_position, const double avg_dt){
    // Compute the change in displacement.
    const double delta_x = robot_velocity.x * avg_dt;
    const double delta_th = robot_velocity.th * avg_dt;

    // Compute the overall displacement.
    robot_position.x += delta_x * cos(robot_position.th);
    robot_position.y += delta_x * sin(robot_position.th);
    robot_position.th += delta_th;
    robot_position.th = fmod(robot_position.th, 2*M_PI);
}

int main(int argc, char** argv) {
    // Initialize ROS.
    ros::init(argc, argv, "base_odom");
    ROS_INFO_STREAM("*Node initiated");
    ros::NodeHandle ros_node;
    ros::Publisher odom_pub = ros_node.advertise<nav_msgs::Odometry>("/base_control/odom", 60);
    ros::Subscriber rr_enc = ros_node.subscribe("/base_control/encoders", 100, halCallBack);
    //ros::Subscriber subImu = ros_node.subscribe("/sensor/imu", 100, imuCallBack);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time = ros::Time::now();
    
    // Checks the loop at a rate of 25Hz -> 0.04 seconds.
    ros::Rate r(25.0);

    // Initialize odometry message.
    nav_msgs::Odometry odom;
    initializeOdometryMessage(odom);

    // Initialize odometry transform.
    geometry_msgs::TransformStamped odom_trans;
    initializeOdometryTransform(odom_trans);

    // Initialize robot velocity variables.
    Velocity robot_velocity;

    // Initialize robot displacement variables.
    Position robot_displacement;

    while (ros_node.ok()) {
        // Saves the current system time.
        current_time = ros::Time::now();

        // Calculates the average time between encoder counts for the front and back encoders.
        const double avg_dt = dt;

        // Skips the rest of the loop if for some reason no time has passed between encoder counts.
        if (avg_dt == 0) {
            continue;
        }
        
        // Compute robot overall velocity.
        computeRobotVelocity(robot_velocity);

        // Compute robot overall displacement.
        computeRobotDisplacement(robot_velocity, robot_displacement, avg_dt);

        // Create quaternion created from yaw.
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_displacement.th);
        
        // Update odometry message stamp.
        odom.header.stamp = current_time;
        
        // Update odometry transform stamp.
        odom_trans.header.stamp = current_time;
        
        // Update odometry message position.
        odom.pose.pose.position.x = robot_displacement.x;
        odom.pose.pose.position.y = robot_displacement.y;
        odom.pose.pose.orientation = odom_quat;
        
        // Update odometry transform position.
        odom_trans.transform.translation.x = robot_displacement.x;
        odom_trans.transform.translation.y = robot_displacement.y;
        odom_trans.transform.rotation = odom_quat;

        // Update odometry velocity.
        odom.twist.twist.linear.x = robot_velocity.x;
        odom.twist.twist.linear.y = robot_velocity.y;
        odom.twist.twist.angular.z = robot_velocity.th;

        // Publish odometry message.
        if (publish_odom_message) {
            odom_pub.publish(odom);
        }

        // Publish odometry transform.
        if (publish_odom_frame) {
            odom_broadcaster.sendTransform(odom_trans);
        }

        ros::spinOnce();
        r.sleep();
    }
}
