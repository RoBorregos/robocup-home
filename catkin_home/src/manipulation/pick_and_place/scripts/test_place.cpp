#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

ros::Publisher pub_place_position;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
}

void cloudCB(const sensor_msgs::PointCloud2& input){
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); // Indices that correspond to a plane.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(input, *cloud);
    computeNormals(cloud, cloud_normals);

    // Do Plane Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000); /* run at max 1000 iterations before giving up */
    segmentor.setDistanceThreshold(0.02); /* tolerance for variation from model */
    segmentor.setInputCloud(cloud);
    
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    
    /* Extract the planar inliers from the input cloud save them in planeCloud */
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(false);
    extract_indices.filter(*planeCloud);

    /* Extract the planar inliers from the input cloud */
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    geometry_msgs::Point place_position;
    place_position.x = centroid[0];
    place_position.y = centroid[1];
    place_position.z = centroid[2];

    pub_place_position.publish(place_position);    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_place");
    ros::NodeHandle nh;

    ros::Subscriber sub_octomap = nh.subscribe("/octomap_binary", 1, octomapCallback);
    pub_place_position = nh.advertise<geometry_msgs::Point>("/place_position", 1);

    ros::spin();
    return 0;
}