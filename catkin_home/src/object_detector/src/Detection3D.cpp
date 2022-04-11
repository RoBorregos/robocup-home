#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/auto_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/convex_hull.h>

#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

struct ObjectParams
{
  /* PointCloud Cluster. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
  /* Mesh. */
  shape_msgs::Mesh mesh;
  /* Center point of the Cluster. */
  geometry_msgs::Pose center;
  /* Height of the Cluster. */
  double height;
};

struct PlaneParams
{
  /* ax + by + cz = d */
  double direction_vec[4];
  /* Center point of the plane. */
  double center_pt[3];
  /* World Z Value. */
  double real_z;
  /* Width of the plane. */
  double width;
  /* Height of the plane. */
  double height;
};

const float can_height = 0.109;
const float can_radius = 0.075 / 2;

class Detect3D
{
  const std::string name = "Detect3D";
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  actionlib::SimpleActionServer<object_detector::DetectObjects3DAction> as_;
  object_detector::DetectObjects3DFeedback feedback_;
  object_detector::DetectObjects3DResult result_;

public:
  Detect3D() :
    listener_(buffer_),
    as_(nh_, name, boost::bind(&Detect3D::handleActionServer, this, _1), false)
  {
    as_.start();
    ROS_INFO_STREAM("Action Server Detect3D - Initialized");
  }

  /** \brief Handle Action Server Goal Received. */
  void handleActionServer(const object_detector::DetectObjects3DGoalConstPtr &goal)
  {
    ROS_INFO_STREAM("Action Server Detect3D - Goal Received");
    feedback_.status = 0;
    result_.objects_found = 0;
    result_.objects_poses.clear();
    result_.objects_names.clear();
    result_.x_plane = 0;
    result_.y_plane = 0;
    result_.z_plane = 0;
    result_.width_plane = 0;
    result_.height_plane = 0;

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM("Action Server Detect3D - Preempted");
      as_.setPreempted(); // Set the action state to preempted
      return;
    }

    boost::shared_ptr<sensor_msgs::PointCloud2 const> input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/xtion/depth_registered/points", nh_);
    Detect3D::cloudCB(input_cloud);
    as_.setSucceeded(result_);
  }

  /** \brief Given the parameters of the object add it to the planning scene. */
  void addObject(const int id, const ObjectParams& object_found)
  {
    // Adding Object to Planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "camera_depth_frame";
    collision_object.id = "object-" + std::to_string(id);

    collision_object.meshes.push_back(object_found.mesh);
    collision_object.mesh_poses.push_back(object_found.center);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    geometry_msgs::PoseStamped object_pose_stamped;
    object_pose_stamped.header.stamp = ros::Time::now();
    object_pose_stamped.header.frame_id = collision_object.header.frame_id;
    object_pose_stamped.pose = object_found.center;
    result_.objects_poses.push_back(object_pose_stamped);
    result_.objects_names.push_back(collision_object.id);
  }

  /** \brief Given the parameters of the plane add it to the planning scene. */
  bool addPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients_plane)
  {
    PlaneParams plane_params;
    plane_params.direction_vec[0] = coefficients_plane->values[0];
    plane_params.direction_vec[1] = coefficients_plane->values[1];
    plane_params.direction_vec[2] = coefficients_plane->values[2];
    plane_params.direction_vec[3] = coefficients_plane->values[3];
    extractPlaneDetails(cloud, plane_params);

    // Adding Plane to Planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = "table";

    // Define a object which will be added to the world.
    shape_msgs::SolidPrimitive solid_primitive;
    solid_primitive.type = solid_primitive.BOX;
    solid_primitive.dimensions.resize(3);
    solid_primitive.dimensions[0] = plane_params.width;
    solid_primitive.dimensions[1] = plane_params.height;
    solid_primitive.dimensions[2] = 0.02; // Custom table wide.

    // Define a pose for the plane (specified relative to frame_id).
    geometry_msgs::PoseStamped plane_pose;
    plane_pose.header.stamp = ros::Time::now();
    plane_pose.header.frame_id = "camera_depth_frame";
    plane_pose.pose.position.x = plane_params.center_pt[0];
    plane_pose.pose.position.y = plane_params.center_pt[1];
    plane_pose.pose.position.z = plane_params.center_pt[2];
    plane_pose.pose.orientation.x = 0.0;
    plane_pose.pose.orientation.y = 0.0;
    plane_pose.pose.orientation.z = 0.0;
    plane_pose.pose.orientation.w = 1;

    geometry_msgs::PoseStamped pose_transformed;
    
    geometry_msgs::TransformStamped tf_to_map;
    tf_to_map = buffer_.lookupTransform("map", plane_pose.header.frame_id, ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(plane_pose, plane_pose, tf_to_map);

    plane_pose.pose.position.z = plane_params.real_z - solid_primitive.dimensions[2] / 2;
    ROS_INFO_STREAM("Plane Detected at z:" << plane_pose.pose.position.z);
    plane_pose.pose.orientation.x = 0.0;
    plane_pose.pose.orientation.y = 0.0;
    plane_pose.pose.orientation.z = 0.0;
    plane_pose.pose.orientation.w = 1;

    // Ignore it if floor is the plane detected.
    if (plane_pose.pose.position.z < 0.2) {
      return false;
    }

    // Add Plane as Collision Object.
    collision_object.primitives.push_back(solid_primitive);
    collision_object.primitive_poses.push_back(plane_pose.pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    result_.x_plane = plane_pose.pose.position.x;
    result_.y_plane = plane_pose.pose.position.y;
    result_.z_plane = plane_pose.pose.position.z;
    result_.width_plane = plane_params.width;
    result_.height_plane = plane_params.height;
    return true;
  }

  template<typename T>
  void toPoint(const T &in, geometry_msgs::Point &out)
  {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
  }

  /** \brief Given the pointcloud containing just the object, reconstruct a mesh from it.
      @param cloud - point cloud containing just the object. */
  void reconstructMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, shape_msgs::Mesh &mesh)
  {
    pcl::PointCloud<pcl::PointXYZ> output_cloud;
    std::vector<pcl::Vertices> triangles;

    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    indices->resize(cloud->points.size ());
    for (size_t i = 0; i < indices->size (); ++i) { (*indices)[i] = i; }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setInputCloud(cloud);
    mls.setIndices(indices);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    
    mls.process(*mls_normals);
    
    pcl::ConvexHull<pcl::PointXYZ> ch;
    
    ch.setInputCloud(mls_points);
    ch.reconstruct(output_cloud, triangles);

    mesh.vertices.resize(output_cloud.points.size());
    for(size_t i=0; i<output_cloud.points.size(); i++)
     toPoint(output_cloud.points[i], mesh.vertices[i]);
 
    ROS_INFO("Found %ld polygons", triangles.size());
    BOOST_FOREACH(const pcl::Vertices polygon, triangles)
    {
      if(polygon.vertices.size() < 3)
      {
        ROS_WARN("Not enough points in polygon. Ignoring it.");
        continue;
      }
  
      shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
      boost::array<uint32_t, 3> xyz = {{polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]}};
      triangle.vertex_indices = xyz;
  
      mesh.triangles.push_back(shape_msgs::MeshTriangle());
    }
  }

  /** \brief Given the pointcloud containing just the object,
      compute its center point, its height and its mesh and store in object_found.
      @param cloud - point cloud containing just the object. */
  void extractObjectDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ObjectParams& object_found)
  {
    double max_angle_y = -std::numeric_limits<double>::infinity();
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3] = { 0.0, 0.0, 0.0 };
    double highest_point[3] = { 0.0, 0.0, 0.0 };
    
    /*
      Consider a point inside the point cloud and imagine that point is formed on a XY plane where the perpendicular
      distance from the plane to the camera is Z.
      The perpendicular drawn from the camera to the plane hits at center of the XY plane.
      We have the x and y coordinate of the point which is formed on the XY plane.
      X is the horizontal axis and Y is the vertical axis.
      C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane.
      Now we know Z is the perpendicular distance from the point to the camera.
      If you need to find the  actual distance d from the point to the camera, you should calculate the hypotenuse-
      hypot(point.z, point.x);
      angle the point made horizontally atan2(point.z,point.x);
      angle the point made Vertically atan2(point.z, point.y);
    */
    for (auto const point : cloud->points)
    {
      const double angle = atan2(point.z, point.y);
      // Find the coordinates of the highest point.
      if (angle < min_angle_y)
      {
        min_angle_y = angle;
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      // Find the coordinates of the lowest point.
      else if (angle > max_angle_y)
      {
        max_angle_y = angle;
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    // Store the object pose.
    object_found.center.position.x = (highest_point[0] + lowest_point[0]) / 2;
    object_found.center.position.y = (highest_point[1] + lowest_point[1]) / 2;
    object_found.center.position.z = (highest_point[2] + lowest_point[2]) / 2;

    // Store the height of object.
    object_found.height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));
    
    // Store cluster;
    object_found.cluster = cloud;
    reconstructMesh(cloud, object_found.mesh);
  }

  /** \brief Given the pointcloud containing just the plane,
      compute its center point and its width & height and store in plane_found.
      @param cloud - point cloud containing just the plane. */
  void extractPlaneDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, PlaneParams& plane_found)
  {
    double max_x_ = cloud->points[0].x;
    double min_x_ = cloud->points[0].x;
    double max_y_ = cloud->points[0].y;
    double min_y_ = cloud->points[0].y;
    double totalZ = 0.0;
    for (auto const point : cloud->points)
    {
      max_x_ = fmax(max_x_, point.x);
      min_x_ = fmin(min_x_, point.x);
      max_y_ = fmax(max_y_, point.y);
      min_y_ = fmin(min_y_, point.y);
      totalZ += point.z;
    }
    double averageZ = totalZ / cloud->points.size();

    // Store the center point of plane.
    plane_found.center_pt[0] = (max_x_ + min_x_) / 2;
    plane_found.center_pt[1] = (max_y_ + min_y_) / 2;
    plane_found.center_pt[2] = averageZ;

    // Store the measures of plane.
    plane_found.height = std::abs(max_x_ - min_x_) + 0.15;
    plane_found.width = std::abs(max_y_ - min_y_) + 0.15;

    // Get Z Value Referenced To Map
    geometry_msgs::PoseStamped pose_transformed;
    geometry_msgs::TransformStamped tf_to_map;

    geometry_msgs::PoseStamped plane_pose;
    plane_pose.header.stamp = ros::Time::now();
    plane_pose.header.frame_id = "camera_depth_frame";
    plane_pose.pose.position.x = cloud->points[0].x;
    plane_pose.pose.position.y = cloud->points[0].y;
    plane_pose.pose.position.z = cloud->points[0].z;
    plane_pose.pose.orientation.x = 0.0;
    plane_pose.pose.orientation.y = 0.0;
    plane_pose.pose.orientation.z = 0.0;
    plane_pose.pose.orientation.w = 1;

    tf_to_map = buffer_.lookupTransform("map", plane_pose.header.frame_id, ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(plane_pose, plane_pose, tf_to_map);
    plane_found.real_z  = plane_pose.pose.position.z;
  }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 1.3);
    pass.filter(*cloud);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
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

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);
  }

  /** \brief Given the pointcloud remove the biggest plane from the pointcloud. Return True if its the table.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. (output) */
  bool removeBiggestPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane)
  {
    // Do Plane Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000); /* run at max 1000 iterations before giving up */
    segmentor.setDistanceThreshold(0.01); /* tolerance for variation from model */
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

    bool isTheTable = false;
    isTheTable = addPlane(planeCloud, coefficients_plane);
    if (isTheTable) {
      pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
    }
    return isTheTable;
  }
  
  /** \brief PointCloud callback. */
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // Get cloud ready
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); // Indices that correspond to a plane.
    pcl::fromROSMsg(*input, *cloud);
    passThroughFilter(cloud);
    computeNormals(cloud, cloud_normals);

    // Detect and Remove the table on which the object is resting.
    bool tableRemoved = false;
    while(!tableRemoved && !cloud->points.empty()) {
      tableRemoved = removeBiggestPlane(cloud, inliers_plane);
      extractNormals(cloud_normals, inliers_plane);
    }

    /* Extract all objects from PointCloud using Clustering. */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    getClusters(cloud, 0.05, clusters);
    int clustersFound = clusters.size();

    std::vector<ObjectParams> objects(clustersFound);
    for(int i=0;i < clustersFound; i++) {
      extractObjectDetails(clusters[i], objects[i]);
      addObject(i, objects[i]);
    }
  }

  /** \brief Find all clusters in a pointcloud.*/
  void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double tolerance, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*

      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);
    }
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "detect_objects_and_table");

  // Start the segmentor
  Detect3D segmentor;

  // Spin
  ros::spin();
}
