#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pickAndPlace");


  tf2::Matrix3x3 r(1.0, 1.0, 1.0, 2.0, 2.0, 2.0,
                  0.0, 0.0, 0.0);

  /*
  numpy.array([],[]

  */
  tf2::Quaternion quat;
  r.getRotation(quat);

  ROS_INFO_STREAM(quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW());
  // EEF yaw-offset to its parent-link (last link of arm)
  quat *= tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.78);

  ROS_INFO_STREAM(quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW());
  quat.normalize();

  ROS_INFO_STREAM(quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW());
  // geometry_msgs::Quaternion tmp = tf2::toMsg(quat);
  // ROS_INFO_STREAM(tmp.x << " " << tmp.y << " " << tmp.z << " " << tmp.w);
  return 0;
}
