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

#include <object_detector/objectDetectionArray.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudSources.h>

#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


#include <std_msgs/Int64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#define CAMERA_FRAME "head_rgbd_sensor_depth_frame"

using namespace octomap;

void save_octomap(const moveit_msgs::PlanningSceneConstPtr& input) {
  // Extracting the MoveIt! planning scene world published by /move_group
  moveit_msgs::PlanningScene::Ptr my_planning_scene(new moveit_msgs::PlanningScene);
  *my_planning_scene = *input;
  moveit_msgs::PlanningSceneWorld my_world = (*my_planning_scene).world;

  // Extracting only the OctoMap
  octomap_msgs::OctomapWithPose octomap_pose = my_world.octomap;
  octomap_msgs::Octomap octomap = octomap_pose.octomap;

  // Conversion from octomap_msgs to octomap
  AbstractOcTree* my_abstract_map = octomap_msgs::msgToMap(octomap);

  // Obtaining the actual OctoMap tree
  OcTree* my_map = (OcTree*)my_abstract_map;
  OcTree tree = *my_map;

  tree.writeBinary("my_tree.bt"); //if you want to save the OcTree in a file
}


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "detect_objects_and_table");

  ros::NodeHandle nh_;
  boost::shared_ptr<moveit_msgs::PlanningScene const> t_ps;
  t_ps = (ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", nh_));
  save_octomap(t_ps);
}
