#ifndef POSEESTIMATIONHANDLER_H_
#define POSEESTIMATIONHANDLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CompressedImage.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class PoseEstimationHandler {
	public:
		PoseEstimationHandler(ros::NodeHandle& pose_estimation_node) : it_(pose_estimation_node) {
			ROS_INFO("Subscribing to IntelCamera image node...");
			intel_cam_sub_ = it_.subscribe(
				"/camera/color/image_raw", 1, &PoseEstimationHandler::imageCompressedCallback, this);

			image_pub_ = it_.advertise("/pose_estimation_node/output_video", 1);
			cv::namedWindow(OPENCV_WINDOW);
			ROS_INFO("Subscriber initialized!");
		}

		~PoseEstimationHandler() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

	private:
		void imageCompressedCallback(const sensor_msgs::ImageConstPtr& event) {
			// TODO: Process ROS Image to get object pose estimation
			displayCVImage(event);
		}

		void displayCVImage(const sensor_msgs::ImageConstPtr& event) {
			cv_bridge::CvImagePtr cv_ptr;
			try {
				cv_ptr = cv_bridge::toCvCopy(event, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
			}

			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			cv::waitKey(3);
			image_pub_.publish(cv_ptr->toImageMsg());
		}

		image_transport::ImageTransport it_;
  	image_transport::Subscriber intel_cam_sub_;
  	image_transport::Publisher image_pub_;
};

#endif
