
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
#include <queue>
#include <vector>
#include <math.h>

#include <object_detector/objectDetectionArray.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudIndexed.h>
#include <gpd_ros/CloudSources.h>

#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>
#include <object_detector/GetPlacePositionAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>
#include <set>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace octomap;

#define ENABLE_RANSAC true

struct PointXYZComparator
{
    bool operator()(const pcl::PointXYZ &lhs, const pcl::PointXYZ &rhs) const
    {
        if (lhs.x < rhs.x)
            return true;
        if (lhs.x > rhs.x)
            return false;
        if (lhs.y < rhs.y)
            return true;
        if (lhs.y > rhs.y)
            return false;
        if (lhs.z < rhs.z)
            return true;
        return false;
    }
};

struct ObjectParams
{
    /* PointCloud Cluster. */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
    /* PointCloud Cluster Original. */
    pcl::PointCloud<pcl::PointXYZ> cluster_original;
    /* Mesh. */
    shape_msgs::Mesh::Ptr mesh;
    /* Center point of the Cluster. */
    geometry_msgs::PoseStamped center;
    /* World Z Min Value. */
    double min_z;
    /* World Z Max Value. */
    double max_z;
    /* World X Min Value. */
    double min_x;
    /* World X Max Value. */
    double max_x;
    /* World Y Min Value. */
    double min_y;
    /* World Y Max Value. */
    double max_y;
    /* Detection Label. */
    int label = -1;
    /* Valid Object. */
    bool isValid = false;
    /* Valid Object. */
    int file_id = 0;
};

class Detect3DPlace
{
    std::string POINT_CLOUD_TOPIC = std::string("/head_rgbd_sensor/depth_registered/points");
    std::string BASE_FRAME = std::string("base_link");
    std::string CAMERA_FRAME = std::string("head_rgbd_sensor_depth_frame");
    const std::string name = "Detect3DPlace";
    ros::NodeHandle nh_;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;
    actionlib::SimpleActionServer<object_detector::GetPlacePositionAction> as_; // TODO: Type of simple as
    object_detector::GetPlacePositionFeedback feedback_;
    object_detector::GetPlacePositionResult result_;
    object_detector::objectDetection force_object_;
    int side_ = 0; // -1 left, 0 undefined, 1 right
    bool ignore_moveit_ = true;
    ros::Publisher pose_pub_;
    geometry_msgs::PoseArray pose_pub_msg_;
    gpd_ros::CloudSamples gpd_msg_;
    gpd_ros::CloudIndexed gpd_msg_indexed_;
    ros::Publisher pc_pub_1_;
    ros::Publisher pc_pub_2_;
    ros::ServiceClient clear_octomap;
    ros::Subscriber width_sub_;
    bool biggest_object_;
    float plane_min_height_;
    float plane_max_height_;

    int prefix[29][29] = {0};
    double selected_object_sz = 0.15; //meters
    double grid_size = 0.05; //meters
    const int matrix_size = 29;

    bool robot_range[29][29] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };

    bool possible_placings[29][29];

public:
    Detect3DPlace() : 
        listener_(buffer_),
        as_(nh_, "detect3d_place", boost::bind(&Detect3DPlace::handleActionServer, this, _1), false)
    {
        width_sub_ = nh_.subscribe<std_msgs::Float64>("/pick_width", 1, &Detect3DPlace::pickWidthCb, this);
        planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();
        clear_octomap = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
        clear_octomap.waitForExistence();

        tf_listener = new tf::TransformListener();
        as_.start();
        pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/test/objectposes", 10);
        pc_pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_1", 10);
        pc_pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_2", 10);
        ROS_INFO("Waiting for Clear Octomap service to start.");
        ROS_INFO_STREAM("Action Server Detect3D - Initialized");
        // Load Params.
        nh_.param("/Detection3D/BASE_FRAME", BASE_FRAME, BASE_FRAME);
        nh_.param("/Detection3D/CAMERA_FRAME", CAMERA_FRAME, CAMERA_FRAME);
        nh_.param("/Detection3D/POINT_CLOUD_TOPIC", POINT_CLOUD_TOPIC, POINT_CLOUD_TOPIC);
        ROS_INFO_STREAM("BASE_FRAME: " << BASE_FRAME);
        ROS_INFO_STREAM("CAMERA_FRAME: " << CAMERA_FRAME);
        ROS_INFO_STREAM("POINT_CLOUD_TOPIC: " << POINT_CLOUD_TOPIC);
    }

    void pickWidthCb(const std_msgs::Float64::ConstPtr &msg){
        selected_object_sz = msg->data;
    }

    /** \brief Handle Action Server Goal Received. */
    void handleActionServer(const object_detector::GetPlacePositionGoalConstPtr &goal){
        ROS_INFO_STREAM("Action Server Detect3D - Goal Received");
        ignore_moveit_ = goal->ignore_moveit;
        plane_min_height_ = goal->plane_min_height;
        plane_max_height_ = goal->plane_max_height;
        ROS_INFO_STREAM("Ignore Moveit: " << ignore_moveit_);
        feedback_.status = 0;
        result_.success = true;
        result_.target_pose = geometry_msgs::PoseStamped();
        result_.x_plane = 0;
        result_.y_plane = 0;
        result_.z_plane = 0;
        result_.width_plane = 0;
        result_.height_plane = 0;
        pose_pub_msg_.poses.clear();

        gpd_msg_.cloud_sources = gpd_ros::CloudSources();
        gpd_msg_.samples.clear();

        gpd_msg_indexed_.cloud_sources = gpd_ros::CloudSources();
        gpd_msg_indexed_.indices.clear();

        //reset matrices
        memcpy(possible_placings, robot_range, sizeof(robot_range));
        memset(prefix, 0, sizeof(prefix));

        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO_STREAM("Action Server Detect3D - Preempted");
            as_.setPreempted(); // Set the action state to preempted
            return;
        }

        // Get PointCloud and Transform it to Map Frame
        sensor_msgs::PointCloud2 pc;
        sensor_msgs::PointCloud2 t_pc;
        pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(POINT_CLOUD_TOPIC, nh_));
        if (pc.header.frame_id != BASE_FRAME && tf_listener->canTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0)))
        {
            tf_listener->waitForTransform(BASE_FRAME, CAMERA_FRAME, ros::Time(0), ros::Duration(5.0));
            pc.header.frame_id = CAMERA_FRAME;
            pcl_ros::transformPointCloud(BASE_FRAME, pc, t_pc, *tf_listener);
        }
        else
        {
            t_pc = pc;
        }

        Detect3DPlace::cloudCB(t_pc);
        as_.setSucceeded(result_);
        pose_pub_.publish(pose_pub_msg_);
    }


    void addTable2Possible(const pcl::PointCloud<pcl::PointXYZ>::Ptr &table){
        bool tmp_table[matrix_size][matrix_size] = {false};

        for(auto const point : table->points){
            int y_id = (matrix_size/2) + int(point.y/grid_size);
            int x_id = (matrix_size/2) - int(point.x/grid_size);
            if(y_id<0 || y_id>matrix_size || x_id<0 || x_id>matrix_size )
                continue;
            tmp_table[y_id][x_id] = true;
        }

        ROS_INFO_STREAM("Tmp table Objects");
        for(int i=0; i<matrix_size; i++){
            for(int j=0; j<matrix_size; j++){
                std::cout << tmp_table[i][j] << " ";
            }
            std::cout << std::endl;
        }

        for(int i=0; i<matrix_size; i++)
            for(int j=0; j<matrix_size; j++)
                possible_placings[i][j] &= tmp_table[i][j];

    }

    void addObjPossible(const pcl::PointCloud<pcl::PointXYZ>::Ptr &obj, ObjectParams& obj_params){
        for(auto const point : obj->points){
            int y_id = (matrix_size/2) + int(point.y/grid_size);
            int x_id = (matrix_size/2) - int(point.x/grid_size);
            if(y_id<0 || y_id>matrix_size || x_id<0 || x_id>matrix_size )
                continue;
            possible_placings[y_id][x_id] = false;
        }

        int obj_size_x = std::ceil( (obj_params.max_x - obj_params.min_x) / grid_size );
        int obj_size_y = std::ceil( (obj_params.max_y - obj_params.min_y) / grid_size );
        ROS_INFO_STREAM("Object Size: " << obj_size_x << " x " << obj_size_y);
        int y_idx = floor( (matrix_size/2) + int(obj_params.min_y/grid_size) );
        int x_idx = floor( (matrix_size/2) - int(obj_params.min_x/grid_size) );

        
        for(int i = y_idx; i<y_idx+obj_size_y; i++){
            for(int j = x_idx; j<x_idx+obj_size_x; j++){
                possible_placings[i][j] = false;
            }
        }
    }

    /** \brief Given the parameters of the plane add it to the planning scene. */
    bool addPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::ModelCoefficients::Ptr &coefficients_plane, ObjectParams &plane_params){
        extractObjectDetails(cloud, plane_params, plane_params, false);

        // Z conditions to know if it is a plane parallel to the robot.
        if (plane_params.max_z - plane_params.min_z > 0.30){ // Diff > 30cm, Not Parallel Plane
            ROS_INFO_STREAM("Plane not parallel to the robot " << plane_params.max_z << " - " << plane_params.min_z << " > 0.15");
            return false;
        }

        if (plane_params.max_z > plane_max_height_){ // Z Value Ex. > than 30cm, Not Floor.
            ROS_INFO_STREAM("Plane height too high. " << plane_params.max_z << " > " << plane_max_height_);
            return false;
        }

        if (plane_params.min_z < plane_min_height_){ // Z Value Ex. < than 30cm, Floor Detected.
            ROS_INFO_STREAM("Plane height too low. " << plane_params.min_z << " < " << plane_min_height_);
            return false;
        }
        
        

        // Adding Plane to Planning Scene
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = BASE_FRAME;
        collision_object.id = "plane";


        // Define a cylinder which will be added to the world.
        shape_msgs::SolidPrimitive solid_primitive;
        solid_primitive.type = solid_primitive.BOX;
        solid_primitive.dimensions.resize(3);
        solid_primitive.dimensions[0] = std::abs(plane_params.max_y - plane_params.min_y) * 2 / 3;
        solid_primitive.dimensions[1] = std::abs(plane_params.max_x - plane_params.min_x) * 2 / 3;
        solid_primitive.dimensions[2] = 0.02; // Custom plane Z.


        geometry_msgs::PoseStamped plane_pose;
        plane_pose.header.stamp = ros::Time::now();
        plane_pose.header.frame_id = BASE_FRAME;
        plane_pose.pose.position.x = (plane_params.max_x + plane_params.min_x) / 2;
        plane_pose.pose.position.y = (plane_params.max_y + plane_params.min_y) / 2;
        plane_pose.pose.position.z = (plane_params.max_z + plane_params.min_z) / 2 - 0.04;
        ROS_INFO_STREAM("Plane Detected at " << std::to_string((plane_params.max_z + plane_params.min_z) / 2));
        plane_pose.pose.orientation.x = 0.0; // atan2(fabs(plane_params.max_x_p.y - plane_params.min_x_p.y), fabs(plane_params.max_x_p.x - plane_params.min_x_p.x));
        plane_pose.pose.orientation.y = 0.0; // atan2(fabs(plane_params.max_y_p.y - plane_params.min_y_p.y), fabs(plane_params.max_y_p.x - plane_params.min_y_p.x));
        plane_pose.pose.orientation.z = 0.0;
        plane_pose.pose.orientation.w = 1;

        collision_object.primitives.push_back(solid_primitive);
        collision_object.primitive_poses.push_back(plane_pose.pose);
        collision_object.operation = collision_object.ADD;
        // TODO: Improve Plane Definition or conitnue using octomap.
        // planning_scene_interface_->applyCollisionObject(collision_object);

        plane_params.isValid = true;
        //reconstructMesh(cloud, object_found.mesh);
        addTable2Possible(cloud);

        return true;
    }

    template <typename T>
    void toPoint(const T &in, geometry_msgs::Point &out){
        out.x = in.x;
        out.y = in.y;
        out.z = in.z;
    }

    /** \brief Given pcl Polygon Mesh convert it to ros Mesh.
        @param polygon_mesh_ptr - Polygon Mesh Pointer. */
    bool convertPolygonMeshToRosMesh(const pcl::PolygonMesh::Ptr polygon_mesh_ptr, shape_msgs::Mesh::Ptr ros_mesh_ptr){
        ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh started.");

        pcl_msgs::PolygonMesh pcl_msg_mesh;

        pcl_conversions::fromPCL(*polygon_mesh_ptr, pcl_msg_mesh);

        sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

        size_t size = pcd_modifier.size();

        ros_mesh_ptr->vertices.resize(size);

        ROS_INFO_STREAM("polys: " << pcl_msg_mesh.polygons.size()
                                  << " vertices: " << pcd_modifier.size());

        sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

        for (size_t i = 0u; i < size; i++, ++pt_iter)
        {
            ros_mesh_ptr->vertices[i].x = pt_iter[0];
            ros_mesh_ptr->vertices[i].y = pt_iter[1];
            ros_mesh_ptr->vertices[i].z = pt_iter[2];
        }

        ROS_INFO_STREAM("Updated vertices");

        ros_mesh_ptr->triangles.resize(polygon_mesh_ptr->polygons.size());

        for (size_t i = 0u; i < polygon_mesh_ptr->polygons.size(); ++i)
        {
            if (polygon_mesh_ptr->polygons[i].vertices.size() < 3u)
            {
                ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
                continue;
            }

            for (size_t j = 0u; j < 3u; ++j)
            {
                ros_mesh_ptr->triangles[i].vertex_indices[j] =
                    polygon_mesh_ptr->polygons[i].vertices[j];
            }
        }
        ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh ended.");
        return true;
    }

    /** \brief Given the pointcloud containing just the object, reconstruct a mesh from it.
        @param cloud - point cloud containing just the object. */
    void reconstructMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, shape_msgs::Mesh::Ptr &mesh)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(0.02); // 2cm

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(*triangles);

        convertPolygonMeshToRosMesh(triangles, mesh);
    }

    /** \brief Given the pointcloud containing just the object,
        compute its center point, its height and its mesh and store in object_found.
        @param cloud - point cloud containing just the object. */
    void extractObjectDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, ObjectParams &object_found, const ObjectParams &plane_params, bool isCluster = true){
        ROS_INFO_STREAM("Extracting Object Details " << cloud->points.size());
        object_found.cluster_original = *cloud;
        double max_z_ = cloud->points[0].z;
        double min_z_ = cloud->points[0].z;
        double max_y_ = cloud->points[0].y;
        double min_y_ = cloud->points[0].y;
        double max_x_ = cloud->points[0].x;
        double min_x_ = cloud->points[0].x;
        for (auto const point : cloud->points){
            max_z_ = fmax(max_z_, point.z);
            min_z_ = fmin(min_z_, point.z);
            max_x_ = fmax(max_x_, point.x);
            min_x_ = fmin(min_x_, point.x);
            max_y_ = fmax(max_y_, point.y);
            min_y_ = fmin(min_y_, point.y);
        }
        object_found.max_z = max_z_;
        object_found.min_z = min_z_;
        object_found.max_x = max_x_;
        object_found.min_x = min_x_;
        object_found.max_y = max_y_;
        object_found.min_y = min_y_;

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*cloud, centroid);

        // Store the object centroid.
        object_found.center.header.frame_id = BASE_FRAME;
        object_found.center.pose.position.x = centroid.x;
        object_found.center.pose.position.y = centroid.y;
        object_found.center.pose.position.z = centroid.z;
        object_found.center.pose.orientation.x = 0.0;
        object_found.center.pose.orientation.y = 0.0;
        object_found.center.pose.orientation.z = 0.0;
        object_found.center.pose.orientation.w = 1.0;

        if (isCluster){
            object_found.isValid = true;

            // Add object only if is above plane.
            if (plane_params.isValid && object_found.center.pose.position.z < plane_params.min_z - 0.025){
                ROS_INFO_STREAM("Object rejected due to plane height: "
                                << " x: " << object_found.center.pose.position.x
                                << " y: " << object_found.center.pose.position.y
                                << " z: " << object_found.center.pose.position.z
                                << " Tz: " << plane_params.min_z);

                object_found.isValid = false;
                return;
            }

            // Change point cloud origin to object centroid and save it.
            for (auto &point : cloud->points){
                point.x = point.x - centroid.x;
                point.y = point.y - centroid.y;
                point.z = point.z - centroid.z;
            }
            object_found.cluster = cloud;

            ROS_INFO_STREAM("Object dimensions: "
                            << " max_x " << object_found.max_x
                            << " min_x " << object_found.min_x
                            << " max_y " << object_found.max_y
                            << " min_y " << object_found.min_y
                            << " max_z " << object_found.max_z
                            << " min_z " << object_found.min_z);
            // Add object only if it has restricted dimensions.
            if (abs(object_found.max_x - object_found.min_x) > 0.5 ||
                abs(object_found.max_y - object_found.min_y) > 0.5 ||
                abs(object_found.max_z - object_found.min_z) > 0.5){
                ROS_INFO_STREAM("Object rejected due to dimensions.");
                object_found.isValid = false;
                return;
            }

            if (abs(object_found.max_z - object_found.min_z) < 0.03){
                ROS_INFO_STREAM("Object rejected due to height.");
                object_found.isValid = false;
                return;
            }

            if (object_found.max_x > plane_params.max_x || object_found.min_x < plane_params.min_x ||
                object_found.max_y > plane_params.max_y || object_found.min_y < plane_params.min_y){
                ROS_INFO_STREAM("Object rejected due to not within table plane ROI.");
                object_found.isValid = false;
                return;
            }

            if (!ignore_moveit_){
                // Store mesh.
                ROS_INFO_STREAM("Reconstructing Mesh Started");
                reconstructMesh(cloud, object_found.mesh);
                ROS_INFO_STREAM("Reconstructing Mesh Ended");

                if (object_found.mesh == nullptr || object_found.mesh->triangles.size() == 0 || object_found.mesh->vertices.size() == 0)
                {
                    ROS_INFO_STREAM("Object rejected due to invalid mesh.");
                    object_found.isValid = false;
                    return;
                }

                // Scale mesh.
                // const double resize_factor = 2.0;
                // for (auto &vertex : object_found.mesh->vertices)
                // {
                //   vertex.x *= resize_factor;
                //   vertex.y *= resize_factor;
                //   vertex.z *= resize_factor;
                // }
            }
        }
    }

    /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
        @param cloud - Pointcloud.
        @param cloud_normals - The point normals once computer will be stored in this. */
    void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals){
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        // Set the number of k nearest neighbors to use for the feature estimation.
        ne.setKSearch(50);
        ne.compute(*cloud_normals);
    }

    /** \brief Given the point normals and point indices, extract the normals for the indices.
        @param cloud_normals - Point normals.
        @param inliers - Indices whose normals need to be extracted. */
    void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, const pcl::PointIndices::Ptr &inliers){
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        extract_normals.setNegative(true);
        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers);
        extract_normals.filter(*cloud_normals);
    }

    /** \brief Given the pointcloud remove the biggest plane from the pointcloud. Return True if its the target plane.
        @param cloud - Pointcloud.
        @param inliers_plane - Indices representing the plane. (output) */
    bool findBiggestPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointIndices::Ptr &inliers_plane, ObjectParams &plane_params, int count){
        // Do Plane Segmentation
        pcl::SACSegmentation<pcl::PointXYZ> segmentor;
        segmentor.setOptimizeCoefficients(true);
        segmentor.setModelType(pcl::SACMODEL_PLANE);
        segmentor.setMethodType(pcl::SAC_RANSAC);
        segmentor.setMaxIterations(1000);     /* run at max 1000 iterations before giving up */
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

        bool isTheTargetPlane = false;

        isTheTargetPlane = addPlane(planeCloud, coefficients_plane, plane_params);
        if (isTheTargetPlane){
            ROS_INFO_STREAM("Table Found at MaxZ:" << plane_params.max_z << ", MinZ:" << plane_params.min_z << "\n");
            pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
            ROS_INFO_STREAM("File saved: "
                            << "pcl_table.pcd");
            pcl::io::savePCDFile("pcl_no_table.pcd", *cloud);
            ROS_INFO_STREAM("File saved: "
                            << "pcl_no_table.pcd");
        }

        if (!isTheTargetPlane){
            count++;
            ROS_INFO_STREAM("Plane Found");
            pcl::io::savePCDFile("pcl_plane_" + std::to_string(count) + ".pcd", *planeCloud);
            ROS_INFO_STREAM("File saved: "
                            << "pcl_plane_" + std::to_string(count) + ".pcd");
        }

        return isTheTargetPlane;
    }

    /** \brief PointCloud callback. */
    void cloudCB(const sensor_msgs::PointCloud2 &input){
        ROS_INFO_STREAM("Received PointCloud");
        if (!ignore_moveit_){
            // Reset Planning Scene Interface
            std::vector<std::string> object_ids = planning_scene_interface_->getKnownObjectNames();
            planning_scene_interface_->removeCollisionObjects(object_ids);
            ros::Duration(2).sleep();
        }

        // Get cloud ready
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); // Indices that correspond to a plane.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(input, *cloud);
        computeNormals(cloud, cloud_normals);

        ROS_INFO_STREAM("PointCloud Pre-Processing Done");

        // Detect the plane of the table
        ObjectParams plane_params;
        if (ENABLE_RANSAC){
            bool targetPlaneFound = false;
            int count = 0;
            while (!targetPlaneFound){ // && cloud->points.size() > 3){
                targetPlaneFound = findBiggestPlane(cloud, inliers_plane, plane_params, count++);
                extractNormals(cloud_normals, inliers_plane);
            }
        }
        
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        getClusters(cloud, clusters);
        int clustersFound = clusters.size();
        ROS_INFO_STREAM("Clusters Found: " << clustersFound);

        std::vector<ObjectParams> objects;
        for (int i = 0; i < clustersFound; i++){
            ObjectParams tmp;
            tmp.mesh.reset(new shape_msgs::Mesh);
            extractObjectDetails(clusters[i], tmp, plane_params);
            tmp.file_id = i;
            ROS_INFO_STREAM("File saved: "
                            << "pcl_object_" + std::to_string(i) + ".pcd");
            pcl::io::savePCDFile("pcl_object_" + std::to_string(i) + ".pcd", tmp.cluster_original);
            if (tmp.isValid){
                objects.push_back(tmp);
                addObjPossible(clusters[i], tmp);
            }
        }
        ROS_INFO_STREAM("Found Objects: " << objects.size());

        //fill area prefix matrix
        for(int i=0; i<matrix_size; i++){
            for(int j=0; j<matrix_size; j++){
                if(possible_placings[i][j]){
                    prefix[i][j] = 1;
                }
                else{
                    prefix[i][j] = 0;
                    continue;
                }
                if(i>0 && j>0)
                    prefix[i][j] += std::min( prefix[i-1][j-1], std::min( prefix[i-1][j], prefix[i][j-1] ) );
                else if(i>0)
                    prefix[i][j] += prefix[i-1][j];
                else if(j>0)
                    prefix[i][j] += prefix[i][j-1];

            } 
        }
        
        //debug table objects and prefix matrices
        ROS_INFO_STREAM("Table Objects");
        for(int i=0; i<matrix_size; i++){
            for(int j=0; j<matrix_size; j++){
                std::cout << possible_placings[i][j] << " ";
            }
            std::cout << std::endl;
        }

        ROS_INFO_STREAM("Area Prefix");
        for(int i=0; i<matrix_size; i++){
            for(int j=0; j<matrix_size; j++){
                std::cout << prefix[i][j] << " ";
            }
            std::cout << std::endl;
        }

        bool table_empty = false;
        if( table_empty ){
        }

        /* Idea for search algorithm:
        Given the max dimension por the package as M, find an area of M*M for the place
        After obtaining plane's max and min positions, the size of the plane is W = max x - min x,
        H = max y - min y.
        Define a boolean matrix of dimensions [W/(M/2)][H/(M/2)] set to false,
        min x and min y would represent the index 0,0
        Find each cluster and validate if its on the table, then, set it's dimensions to true
        in the matrix, in the index corresponding to (obj min x - min x)/(M/2) until its area is covered
        After adding every object to the matrix, compute two prefix matrices of integers:
        One keeping track of the previous continuous available spaces (false) to the left,
        and the other above, if a layout table is like this:

        Table           Prefix horizontal   Vertical
        001000010       120123401           110111101
        010010000       101201234           201202212
        100000010       012345601           012313303
        000001000       123450123           123420414

        As the size was defined to M/2, with a traverse of O(N), starting from right down and 
        moving left (for priority of placing close to the arm), when a 2 of maximum is found in
        both matrices, it would fit the size of the object. */

        struct tile{
            int y;
            int x;
        };
        int x_closests_av = -1;
        int y_closests_av = -1;

        tile current = {14, 14};
        bool visited[29][29] = {false};
        std::vector< std::pair<int, tile> > found_placings;
        std::queue<tile> q;
        q.push(current);

        while( !q.empty() ){
            tile tmp = q.front();
            q.pop();
            //ROS_INFO_STREAM("Checking tile: " << tmp.y << " " << tmp.x);
            if(tmp.y < 0 || tmp.y == matrix_size || tmp.x < 0 || tmp.x == matrix_size)
                continue;
            if(visited[tmp.y][tmp.x])
                continue;
            visited[tmp.y][tmp.x] = true;

            if( prefix[tmp.y][tmp.x] >= selected_object_sz/grid_size ){
                //assign the weight, priority of area, then distance to the center
                int weight = prefix[tmp.y][tmp.x]*9 + (14-sqrt( pow(tmp.y - (matrix_size/2), 2) + pow(tmp.x - (matrix_size/2), 2) ));
                found_placings.push_back({weight, tmp});
            }
            current = {tmp.y-1, tmp.x};
            q.push(current);
            current = {tmp.y, tmp.x+1};
            q.push(current);
            current = {tmp.y, tmp.x-1};
            q.push(current);
            current = {tmp.y+1, tmp.x};
            q.push(current);
        }


        //sort the placings by weight
        int max_weight = 0;
        for(auto const placing : found_placings){
            if(placing.first > max_weight){
                max_weight = placing.first;
                x_closests_av = placing.second.x - 3*(selected_object_sz/grid_size)/4;
                y_closests_av = placing.second.y - 3*(selected_object_sz/grid_size)/4;
            }
        }
        
        if( y_closests_av == -1 || x_closests_av == -1 ){
            ROS_INFO_STREAM("No available space for object");
            result_.success = false;
            return;
        }
        
        double x_pos = -(x_closests_av - (matrix_size/2)) * grid_size;
        double y_pos = (y_closests_av - (matrix_size/2)) * grid_size;

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = BASE_FRAME;
        target_pose.pose = plane_params.center.pose;
        target_pose.pose.position.x = x_pos;
        target_pose.pose.position.y = y_pos;
        pose_pub_msg_.header = target_pose.header;
        pose_pub_msg_.poses.push_back(target_pose.pose);
        result_.target_pose = target_pose;
        
        ROS_INFO_STREAM("Plane params: " << plane_params.min_x << " " << plane_params.max_y);
        ROS_INFO_STREAM("Target matrix position: " << y_closests_av << " " << x_closests_av);
        ROS_INFO_STREAM("Target Pose: " << target_pose.pose.position.x << ", " << target_pose.pose.position.y << ", " << target_pose.pose.position.z);
        return;
    }

    /** \brief Find all clusters in a pointcloud.*/
    void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
    {
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Set parameters for the clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.03); // 3cm
        ec.setMinClusterSize(25);
        ec.setMaxClusterSize(20000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*

            // Discard Noise
            if (cloud_cluster->points.size() < 25)
            {
                continue;
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    }

private:
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    tf::TransformListener *tf_listener;
};

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "detector_place_position");

    // Start the segmentor
    Detect3DPlace segmentor_place;

    // Spin
    ros::spin();
}