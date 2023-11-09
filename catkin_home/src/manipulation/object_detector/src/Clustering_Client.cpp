// Test for the clustering service
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <object_detector/Clustering.h>

#include <iostream>

#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clustering_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<object_detector::Clustering>("Clustering");
  object_detector::Clustering srv;

  // read pcl from file ~/.ros/pcl_table.pcd
  ROS_INFO("Reading pcl from file ~/.ros/pcl_achievable_cloud.pcd");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("/home/rbrgs-pc/.ros/pcl_achievable_cloud.pcd", *cloud);
  // print one point from the cloud
  ROS_INFO("First point in the cloud: x=%f, y=%f, z=%f", cloud->points[0].x, cloud->points[0].y, cloud->points[0].z);

  // convert pcl to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "table_frame";
  cloud_msg.header.stamp = ros::Time::now();
  ROS_INFO("Converted pcl to sensor_msgs::PointCloud2");

  srv.request.pointcloud = cloud_msg;
  srv.request.n_clusters = 4;

  ROS_INFO("Calling service Clustering");

  if (client.call(srv))
  {
    ROS_INFO("Target x: %f", srv.response.x_center);
    ROS_INFO("Target y: %f", srv.response.y_center);
  }
  else
  {
    ROS_ERROR("Failed to call service Clustering");
    return 1;
  }

  return 0;
}

