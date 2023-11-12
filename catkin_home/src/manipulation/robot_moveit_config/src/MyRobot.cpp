#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle nh) : nh(nh)
  {
    // Initialize joint state
    joint_position_[0] = 0.0;
    joint_velocity_[0] = 0.0;
    joint_effort_[0] = 0.0;
    joint_position_[1] = 0.0;
    joint_velocity_[1] = 0.0;
    joint_effort_[1] = 0.0;
    joint_position_[2] = 0.0;
    joint_velocity_[2] = 0.0;
    joint_effort_[2] = 0.0;

    // Register joint interfaces
    hardware_interface::JointStateHandle state_handle_1("odom_x", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("odom_y", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    hardware_interface::JointStateHandle state_handle_3("odom_r", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    jnt_state_interface_.registerHandle(state_handle_3);

    hardware_interface::JointHandle pos_handle_1(jnt_state_interface_.getHandle("odom_x"), &joint_position_command_[0]);
    jnt_pos_interface_.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface_.getHandle("odom_y"), &joint_position_command_[1]);
    jnt_pos_interface_.registerHandle(pos_handle_2);

    hardware_interface::JointHandle pos_handle_3(jnt_state_interface_.getHandle("odom_r"), &joint_position_command_[2]);
    jnt_pos_interface_.registerHandle(pos_handle_3);

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);

    pubTwist = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
    pubJoint = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    prev_time_ = ros::Time::now();
  }

  void read() {
    // Calculate velocity from change in position over time
    ros::Time time = ros::Time::now();
    ros::Duration period = time - prev_time_;
    prev_time_ = time;

    

    joint_velocity_[0] = (joint_position_command_[0] - joint_position_[0]) / period.toSec();
    joint_velocity_[1] = (joint_position_command_[1] - joint_position_[1]) / period.toSec();
    joint_velocity_[2] = (joint_position_command_[2] - joint_position_[2]) / period.toSec();

    if (joint_position_[0] != joint_position_command_[0] || joint_position_[1] != joint_position_command_[1] || joint_position_[2] != joint_position_command_[2]) {
      ROS_INFO("%f = (%f - %f) / %f", joint_velocity_[2], joint_position_command_[2], joint_position_[2], period.toSec());
    }

    joint_position_[0] = joint_position_command_[0];
    joint_position_[1] = joint_position_command_[1];
    joint_position_[2] = joint_position_command_[2];
    
  }

  void write() {
    geometry_msgs::Twist twist;
    twist.linear.x = joint_velocity_[0];
    twist.linear.y = joint_velocity_[1];
    twist.angular.z = joint_velocity_[2];
    pubTwist.publish(twist);

    std::vector<std::string> joint_names = {"odom_x", "odom_y", "odom_r"};
    sensor_msgs::JointState joint_state;
    joint_state.header = std_msgs::Header();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    joint_state.position = std::vector<double>(joint_position_, joint_position_ + 3);
    joint_state.velocity = std::vector<double>(joint_velocity_, joint_velocity_ + 3);
    joint_state.effort = std::vector<double>(joint_effort_, joint_effort_ + 3);
    pubJoint.publish(joint_state);
  } 

private:
  ros::NodeHandle nh;
  ros::Publisher pubTwist;
  ros::Publisher pubJoint;
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  double joint_position_[3];
  double joint_velocity_[3];
  double joint_effort_[3];
  double joint_position_command_[3];
  ros::Time prev_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_robot_hw");
  ros::NodeHandle nh;

  ROS_INFO("Starting my_robot_hw.");
  
  MyRobot robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(4);
	spinner.start();

  ros::Duration(2.0).sleep();

  ROS_INFO("Started my_robot_hw.");

  ros::Rate rate(10);
  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    ros::Time time = ros::Time::now();
    ros::Duration period = time - prev_time;
    prev_time = time;

    robot.read();
    cm.update(time, period);
    robot.write();

    rate.sleep();
  }

  spinner.stop();
  return 0;
}
