#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

class OctomapService
{
public:
  OctomapService()
  {
    tf_listener = new tf::TransformListener();
    state_ = true;
    sub_ = nh_.subscribe("/zed2/zed_node/point_cloud/ds_cloud_registered", 1, &OctomapService::cloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/octomap/points", 10);
    service_ = nh_.advertiseService("/toggle_octomap", &OctomapService::toggleOctomap, this);
    state_pub_ = nh_.advertise<std_msgs::Bool>("/octomap/state", 10);
  }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0, 2.5);
    pass.filter(*cloud);
  }

  void cloudCallback(const sensor_msgs::PointCloud2& data)
  {
    if (state_)
    {
      sensor_msgs::PointCloud2 t_pc;
      try {
        tf_listener->waitForTransform("base_link", "zed2_left_camera_frame", ros::Time(0), ros::Duration(5.0));
      } catch (tf::TransformException ex) {
        return;
      }
      pcl_ros::transformPointCloud("base_link", data, t_pc, *tf_listener);
      t_pc.header.frame_id = "base_link";
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(t_pc, *cloud);
      // passThroughFilter(cloud);
      pcl::toROSMsg(*cloud, t_pc);
      pub_.publish(t_pc);
    }
  }

  bool toggleOctomap(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
  {
    state_ = req.data;
    std_msgs::Bool state_msg;
    state_msg.data = state_;
    state_pub_.publish(state_msg);
    res.success = true;
    res.message = "Octomap state changed to " + std::to_string(state_);
    return true;
  }

private:
  tf::TransformListener *tf_listener;
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceServer service_;
  ros::Publisher state_pub_;
  bool state_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_service_node");
  OctomapService service;
  ros::spin();
  return 0;
}