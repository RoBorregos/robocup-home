#ifndef DEPTHHANDLER_H_
#define DEPTHHANDLER_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

static const std::string TOPIC_DEPTH = "/camera/depth/image_rect_raw";

// This class manages the collection from ros and processing of the PCL
// from the intel D435i camera.
class DepthHandler {
	public:
		DepthHandler(ros::NodeHandle& pe_node) {
			depth_sub_ = pe_node.subscribe(
				TOPIC_DEPTH, 1, &DepthHandler::depthCB, this);
			depth_pub_ = 
				pe_node.advertise<sensor_msgs::Image>("/depth/image", 1);
		}

	private:
		void depthCB(const sensor_msgs::ImageConstPtr& depth_msg) {	
			depth_pub_.publish(depth_msg);
		}

		ros::Subscriber depth_sub_;
		ros::Publisher depth_pub_;
};

#endif
