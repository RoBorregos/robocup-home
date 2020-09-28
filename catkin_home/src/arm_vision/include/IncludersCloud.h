#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <iterator>  // for iterators
#include <vector>    // for vectors
//#include<conio.h>
#include <math.h>
//#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <arm_vision/SegmentedClustersArray.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <obj_recognition/ClusterData.h>

// PCL specific includes
