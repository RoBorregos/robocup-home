#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <xarm_msgs/SetDigitalIO.h>
#include <xarm_msgs/GetDigitalIO.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(ros::NodeHandle nh) : nh(nh)
  {
    pubTwist = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 1);
    pubJoint = nh.advertise<sensor_msgs::JointState>("/DASHGO/joint_states", 1);
    
    const int num_joints = 6;
    
    // Initialize joint state
    gripper_changed_ = true;
    head_joint_position_.assign(2, 0.0);
    head_joint_velocity_.assign(2, 0.0);
    head_joint_effort_.assign(2, 0.0);
    gripper_joint_position_.assign(1, 0.0);
    gripper_joint_velocity_.assign(1, 0.0);
    gripper_joint_effort_.assign(1, 0.0);
    head_joint_position_command_.assign(2, 0.0);
    gripper_joint_position_command_.assign(1, 0.0);



    // Register joint interfaces
    hardware_interface::JointStateHandle state_handles[num_joints] = {
      { "Cam1Rev1", &head_joint_position_[0], &head_joint_velocity_[0], &head_joint_effort_[0] },
      { "Cam1Rev2", &head_joint_position_[1], &head_joint_velocity_[1], &head_joint_effort_[1] },
      { "Rev1Servo", &gripper_joint_position_[0], &gripper_joint_velocity_[0], &gripper_joint_effort_[0] }
    };

    for (int i = 0; i < num_joints; i++) {
      jnt_state_interface_.registerHandle(state_handles[i]);
    }

    hardware_interface::JointHandle pos_handles[num_joints] = {
      { jnt_state_interface_.getHandle("Cam1Rev1"), &head_joint_position_command_[0] },
      { jnt_state_interface_.getHandle("Cam1Rev2"), &head_joint_position_command_[1] },
      { jnt_state_interface_.getHandle("Rev1Servo"), &gripper_joint_position_command_[0] }
    };
    
    for (int i = 0; i < num_joints; i++) {
      jnt_pos_interface_.registerHandle(pos_handles[i]);
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_interface_);

    prev_time_ = ros::Time::now();
  }

  void readHead() {
    head_joint_position_[0] = head_joint_position_command_[0];
    head_joint_position_[1] = head_joint_position_command_[1];
  }

  void readGripper() {
    if (gripper_joint_position_[0] != gripper_joint_position_command_[0]) {
      gripper_changed_ = true;
    }
    gripper_joint_position_[0] = gripper_joint_position_command_[0];
  }

  void read() {
    readHead();
    readGripper();
    std::vector<std::string> joint_names = {"Cam1Rev1", "Cam1Rev2", "Rev1Servo", "Rev2Servo"};
    sensor_msgs::JointState joint_state;
    joint_state.header = std_msgs::Header();
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    joint_state.position = std::vector<double>(head_joint_position_.begin(), head_joint_position_.end());
    joint_state.position.insert(joint_state.position.end(), gripper_joint_position_.begin(), gripper_joint_position_.end());
    joint_state.position.push_back(-1 * joint_state.position[5]);
    joint_state.velocity = std::vector<double>(head_joint_velocity_.begin(), head_joint_velocity_.end());
    joint_state.velocity.insert(joint_state.velocity.end(), gripper_joint_velocity_.begin(), gripper_joint_velocity_.end());
    joint_state.velocity.push_back(-1 * joint_state.velocity[5]);
    joint_state.effort = std::vector<double>(head_joint_effort_.begin(), head_joint_effort_.end());
    joint_state.effort.insert(joint_state.effort.end(), gripper_joint_effort_.begin(), gripper_joint_effort_.end());
    joint_state.effort.push_back(-1 * joint_state.effort[5]);
    pubJoint.publish(joint_state);
  }

  void writeHead() {
    return;
  }

  void writeGripper() {
    // Call gripper service
    ros::ServiceClient gripper_client = nh.serviceClient<xarm_msgs::SetDigitalIO>("/xarm/set_digital_out");
    if (gripper_client.exists() && gripper_changed_) {
      gripper_changed_ = false;
      // Gripper Straight < -0.1
      // Gripper 90 > 0.1
      if (gripper_joint_position_[0] < -0.1) { // Open
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = 1;
        srv.request.value = 0;
        if (!gripper_client.call(srv)) {
          ROS_ERROR("Failed to call service");
        }
      } else {
        xarm_msgs::SetDigitalIO srv;
        srv.request.io_num = 1;
        srv.request.value = 1;
        if (!gripper_client.call(srv)) { // Close
          ROS_ERROR("Failed to call service");
        }
      }
    }
  }

  void write() {
    writeHead();
    writeGripper();
  } 

private:
  ros::NodeHandle nh;
  ros::Publisher pubTwist;
  ros::Publisher pubJoint;
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  std::vector<double> head_joint_position_;
  std::vector<double> head_joint_velocity_;
  std::vector<double> head_joint_effort_;
  std::vector<double> head_joint_position_command_;
  std::vector<double> gripper_joint_position_;
  std::vector<double> gripper_joint_velocity_;
  std::vector<double> gripper_joint_effort_;
  std::vector<double> gripper_joint_position_command_;
  ros::Time prev_time_;
  tf::TransformListener listener;
  bool gripper_changed_;
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
