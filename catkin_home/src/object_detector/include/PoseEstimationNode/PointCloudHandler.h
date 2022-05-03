#ifndef POINTCLOUDHANDLER_H_
#define POINTCLOUDHANDLER_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

static const std::string TOPIC_CP = "/camera/depth/color/points";

// This class manages the collection from ros and processing of the PCL
// from the intel D435i camera.
class PointCloudHandler {
	public:
		PointCloudHandler(ros::NodeHandle& pe_node) {
			pc_sub_ = pe_node.subscribe(
				TOPIC_CP, 1, &PointCloudHandler::pointCloudCB, this);
			pc_pub_ = 
				pe_node.advertise<sensor_msgs::PointCloud2>("/pc/filtered", 1);
			pc_err_pub_ = 
				pe_node.advertise<sensor_msgs::PointCloud2>("/pc/err", 1);
		}

	private:
		void pointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {	
			pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
			pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
			pcl::PCLPointCloud2 cloud_filtered;

			// Convert to PCL
			pcl_conversions::toPCL(*cloud_msg, *cloud);

			// Apply Outlier Removal
			// https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html#statistical-outlier-removal
			pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
			sor.setInputCloud(cloudPtr);
			sor.setMeanK(3/*points*/);
			sor.setStddevMulThresh(0.00001);
			sor.filter(cloud_filtered);

			// Publish filtered point cloud
			sensor_msgs::PointCloud2 output;
			pcl_conversions::moveFromPCL(cloud_filtered, output);
			pc_pub_.publish(output);

			// Retrieve the filtered points
			sor.setNegative(true);
			sor.filter(cloud_filtered);

			pcl_conversions::moveFromPCL(cloud_filtered, output);
			pc_err_pub_.publish(output);
		}

		ros::Subscriber pc_sub_;
		ros::Publisher pc_pub_;
		ros::Publisher pc_err_pub_;
};

#endif
