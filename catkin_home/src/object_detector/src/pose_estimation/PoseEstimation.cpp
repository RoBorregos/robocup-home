#include <PoseEstimationNode/PoseEstimationHandler.h>
#include "ros/ros.h"
int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle n;

	ROS_INFO("Initializing Pose Estimation Node");
	PoseEstimationHandler handler(n);
	ros::spin();
	return 0;
}