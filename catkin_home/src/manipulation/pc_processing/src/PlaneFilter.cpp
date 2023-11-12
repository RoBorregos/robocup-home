#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>


class PlaneFilter
{
public:
    PlaneFilter() : nh_()
    {
        tf_listener = new tf::TransformListener();
        // Set up subscriber and publisher
        sub_ = nh_.subscribe("input/points", 1, &PlaneFilter::cloudCB, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output/points", 1);

        ROS_INFO_STREAM("PointCloud planefilter node initialized.");
    }

    /** \brief Given the pointcloud remove the biggest plane from the pointcloud.
      @param cloud - Pointcloud.
    */
    void removeBiggestPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& planeCloud)
    {
        // Do Plane Segmentation
        pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
        segmentor.setOptimizeCoefficients(true);
        segmentor.setModelType(pcl::SACMODEL_PLANE);
        segmentor.setMethodType(pcl::SAC_RANSAC);
        segmentor.setMaxIterations(1000); /* run at max 1000 iterations before giving up */
        segmentor.setDistanceThreshold(0.01); /* tolerance for variation from model */
        segmentor.setInputCloud(cloud);
        
        /* Create the segmentation object for the planar model and set all the parameters */
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        segmentor.segment(*inliers_plane, *coefficients_plane);
        
        /* Extract the planar inliers from the input cloud save them in planeCloud */
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
        extract_indices.setInputCloud(cloud);
        extract_indices.setIndices(inliers_plane);
        extract_indices.setNegative(false);
        extract_indices.filter(*planeCloud);

        /* Extract the planar inliers from the input cloud */
        extract_indices.setInputCloud(cloud);
        extract_indices.setIndices(inliers_plane);
        extract_indices.setNegative(true);
        extract_indices.filter(*cloud);
    }

    void cloudCB(const sensor_msgs::PointCloud2& input_)
    {
        bool tfFailed = false;
        sensor_msgs::PointCloud2 input;
        if (tf_listener->canTransform("base_link", "zed2_left_camera_frame", ros::Time(0))) {
            tf_listener->waitForTransform("base_link", "zed2_left_camera_frame", ros::Time(0), ros::Duration(1.0));
            pcl_ros::transformPointCloud("base_link", input_, input, *tf_listener);
            input.header.frame_id = "base_link";
        } else {
            tfFailed = true;
            input = input_;
        }

        // Downsample the PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(input, *cloud);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        while (cloud->points.size() > 5) {
            removeBiggestPlane(cloud, planeCloud);
            if (tfFailed) {
                break;
            }
            double max_z_ = planeCloud->points[0].z;
            double min_z_ = planeCloud->points[0].z;
            for (auto const point : planeCloud->points)
            {
              max_z_ = fmax(max_z_, point.z);
              min_z_ = fmin(min_z_, point.z);
            }
            if (max_z_ - min_z_ < 0.10) {
                break;
            }
        }

        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = input.header.frame_id;
        output.header.stamp = input.header.stamp;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf::TransformListener *tf_listener;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_planefilter");

    PlaneFilter node;

    ros::spin();

    return 0;
}