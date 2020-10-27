#include "SegmentationNode.h"

segmentation::segmentation(ros::NodeHandle nh, const std::string& camera_selection) : m_nh_(nh), camera_selection_(camera_selection) {
    // define the subscriber and publisher
    
    if(camera_selection.compare("intel") == 0)
    {
        m_sub_ = m_nh_.subscribe("/camera/depth/color/points", 1, &segmentation::cloud_cb, this);
        ROS_INFO("intel camera selected");

    }
    else if(camera_selection.compare("kinect") == 0)
    {
        m_sub_ = m_nh_.subscribe("/camera/depth_registered/points", 1, &segmentation::cloud_cb, this);
        ROS_INFO("kinect camera selected");
    }
    
    //m_sub_ = m_nh_.subscribe("/camera/depth/color/points", 1, &segmentation::cloud_cb, this);
    m_cluster_pub_ = m_nh_.advertise<arm_vision::SegmentedClustersArray>("pcl_clusters", 1);
    m_pub_ = m_nh_.advertise<sensor_msgs::PointCloud2>("output", 1);
    m_pub2_ = m_nh_.advertise<sensor_msgs::PointCloud2>("output2", 1);
}

void segmentation::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 outputPCL;

    sensor_msgs::PointCloud2 output2;
    pcl::PCLPointCloud2 outputPCL2;

    // Container for original & filtered data
    pcl::PCLPointCloud2::Ptr cloudorigin(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloudorigin);
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudorigin);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud_filtered);

    pcl::PCLPointCloud2::Ptr cloud_filtered2(new pcl::PCLPointCloud2);
    pcl::PassThrough<pcl::PCLPointCloud2> pass2;
    pass2.setInputCloud(cloud_filtered);
    pass2.setFilterFieldName("z");
    pass2.setFilterLimits(0.2, 1.5);
    pass2.filter(*cloud_filtered2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PlannedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    double zplanee = 0;
    pcl::fromPCLPointCloud2(*cloud_filtered2, *temp);
    PlannedCloud = PlanarRANSAC(temp, zplanee);
    arm_vision::SegmentedClustersArray CloudClusters1;
    CloudClusters1 = Clustering(PlannedCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr THEcluster(new pcl::PointCloud<pcl::PointXYZRGB>);

    flag = true;
    double tolerance = 0.05;

    // Line for test
    float dumb[2] = {0.4, 0.03};

    THEcluster = CenteredCluster(PlannedCloud, tolerance, flag, dumb);

    pcl::toPCLPointCloud2(*PlannedCloud, outputPCL);
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

    pcl::toPCLPointCloud2(*THEcluster, outputPCL2);
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL2, output2);

    // Publish the data
    m_pub_.publish(output);
    m_cluster_pub_.publish(CloudClusters1);
    m_pub2_.publish(output2);
}
