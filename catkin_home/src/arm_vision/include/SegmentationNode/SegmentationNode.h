#ifndef SEGMENTATIONNODE_H_
#define SEGMENTATIONNODE_H_

#include <ProcessingCloud/ProcessingCloud.h>

static std::string intelParam = "intel";
static std::string kinectParam = "kinect";

class Segmentation {
 public:
    explicit Segmentation(ros::NodeHandle nh, const std::string& camera_selection);

 private:
    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_;
    ros::Publisher m_pub2_;
    ros::Subscriber m_sub_;
    ros::Publisher m_cluster_pub_;
    std::string camera_selection_;

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

};

#endif