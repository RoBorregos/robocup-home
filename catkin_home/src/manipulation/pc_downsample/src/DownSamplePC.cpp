#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudDownsampler
{
public:
    PointCloudDownsampler() : nh_()
    {
        // Set up subscriber and publisher
        sub_ = nh_.subscribe("input/points", 1, &PointCloudDownsampler::cloudCB, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output/points", 1);

        ROS_INFO_STREAM("PointCloud downsampler node initialized.");
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        // Downsample the PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(input, *cloud_input);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_input);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_output);

        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_output, output);
        output.header.frame_id = input.header.frame_id;
        output.header.stamp = input.header.stamp;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_downsampler");

    PointCloudDownsampler node;

    ros::spin();

    return 0;
}