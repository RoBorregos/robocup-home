#ifndef POSEESTIMATIONHANDLER_H_
#define POSEESTIMATIONHANDLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

class PoseEstimationHandler {
	public:
		PoseEstimationHandler(ros::NodeHandle& pose_estimation_node) {
			intel_cam_sub_ = pose_estimation_node.subscribe(
				"/camera/color/image_raw/compressed", 1000, PoseEstimationCallback);
		}

	private:
		static void PoseEstimationCallback(const std_msgs::String::ConstPtr& msg) {
			ROS_INFO("I heard: [%s]", msg->data.c_str());
		}

		ros::Subscriber intel_cam_sub_;
};

#endif
