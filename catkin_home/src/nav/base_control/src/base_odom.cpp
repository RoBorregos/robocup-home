#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/console.h>
#include "base_control/StampedEncoders.h"


//initialize global variables
int wheel1_new = 0;
int wheel2_new = 0;
int wheel3_new = 0;
int wheel4_new = 0;
double dt_front = 0.000001;
double dt_rear = 0.000001;

//initialize constants
const double wheel_radius = 0.125;
const double wheel_circumference = 0.785;
const double encoder_resolution = 1250*4*20;
const double k = 0.47 + 0.55; //the sum of the distance between the wheel's x-coord and the origin, and the y-coord and the origin
const double dist_per_tick = wheel_circumference / encoder_resolution;

//function to save front encoder data to global variables
void feCallBack(const base_control::StampedEncoders::ConstPtr& msg)
{
  ROS_INFO_STREAM("INFO RECEIVED \n *Front Encoders");
  wheel1_new = msg->encoders.left_wheel;
  wheel2_new = msg->encoders.right_wheel;
  dt_front = msg->encoders.time_delta;
}

//function to save rear encoder data to global variables
void reCallBack(const base_control::StampedEncoders::ConstPtr& msg)
{
  ROS_INFO_STREAM("INFO RECEIVED \n *Rear Encoders");
  wheel3_new = msg->encoders.left_wheel;
  wheel4_new = msg->encoders.right_wheel;
  dt_rear = msg->encoders.time_delta;
}

int main(int argc, char** argv)
{
  //initialize ROS
  ros::init(argc, argv, "base_odom");
  ROS_INFO_STREAM("*Node initiated");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/base_control/odom", 60);
  ros::Subscriber fr_enc = n.subscribe("/base_control/front/encoders", 100, feCallBack);
  ros::Subscriber rr_enc = n.subscribe("/base_control/rear/encoders", 100, reCallBack);
  tf::TransformBroadcaster odom_broadcaster;
  
  //initialize displacement variables
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  //initialize velocity variables
  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  
  ros::Time current_time = ros::Time::now();
  
  //checks the loop at a rate of 25Hz
  ros::Rate r(25.0);
  while(n.ok())
  {
    current_time = ros::Time::now(); //saves the current system time
    
    //calculates the average time between encoder counts for the front and rear encoders
    double avg_dt = (dt_front + dt_rear)/2.0; 

    //skips the rest of the loop if for some reason no time has passed between encoder counts
    if(avg_dt == 0)
    {
        continue;
    }

    //compute the velocities of each wheel
    double v_w1 = (wheel1_new * dist_per_tick)/dt_front;
    double v_w2 = (wheel2_new * dist_per_tick)/dt_rear;
    double v_w3 = (wheel3_new * dist_per_tick)/dt_rear;
    double v_w4 = (wheel4_new * dist_per_tick)/dt_front;

    //compute the overall velocity of the robot
    vx = (wheel_radius/4)*(v_w1+v_w2+v_w3+v_w4);
    vy = (wheel_radius/4)*(v_w1-v_w2-v_w3+v_w4);
    vth = (wheel_radius/(4*k))*(-v_w1+v_w2-v_w3+v_w4);

    //compute the change in displacement
    double delta_x = vx * avg_dt;
    double delta_y = vy * avg_dt;
    double delta_th = vth * avg_dt;

    //compute the overall displacement
    x = x + delta_x;
    y = y + delta_y;
    th = th + delta_th;

    //create quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    /*
    //publish the transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    */

    //publish the odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the covariance
    odom.pose.covariance[0] = 1e3; //0.2;
    odom.pose.covariance[7] = 1e3; //0.2;
    odom.pose.covariance[14] = 1e100;
    odom.pose.covariance[21] = 1e100;
    odom.pose.covariance[28] = 1e100;
    odom.pose.covariance[35] = 1e3; //0.2;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;
    
    //set the covariance
    odom.twist.covariance[0] = 1e3; //0.2;
    odom.twist.covariance[7] = 1e3; //0.2;
    odom.twist.covariance[14] = 1e100;
    odom.twist.covariance[21] = 1e100;
    odom.twist.covariance[28] = 1e100;
    odom.twist.covariance[35] = 1e3; //0.2;

    //publish the message
    odom_pub.publish(odom);
    
    ros::spinOnce();
    r.sleep();
  }
}