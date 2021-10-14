#include <PoseEstimationNode/PoseEstimationHandler.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle n;
	PoseEstimationHandler handler(n);
	ros::spin(); 
	return 0;
}