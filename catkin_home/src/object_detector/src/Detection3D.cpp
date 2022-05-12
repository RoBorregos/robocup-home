#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>
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


#include <object_detector/objectDetectionArray.h>
#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>
#include <limits>

struct ObjectParams
{
  /* PointCloud Cluster. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
  /* Mesh. */
  shape_msgs::Mesh::Ptr mesh;
  /* Center point of the Cluster. */
  geometry_msgs::Pose center;
  /* Detection Label. */
  int label = 0;
};

struct PlaneParams
{
  /* ax + by + cz = d */
  double direction_vec[4];
  /* Center point of the plane. */
  double center_pt[3];
  /* World Z Value. */
  double world_z;
  /* Width of the plane. */
  double width;
  /* Height of the plane. */
  double height;
};

class Detect3D
{
  const std::string name = "Detect3D";
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  actionlib::SimpleActionServer<object_detector::DetectObjects3DAction> as_;
  object_detector::DetectObjects3DFeedback feedback_;
  object_detector::DetectObjects3DResult result_;
  ros::Publisher pose_pub_;
  geometry_msgs::PoseArray pose_pub_msg_;
public:
  Detect3D() :
    listener_(buffer_),
    as_(nh_, name, boost::bind(&Detect3D::handleActionServer, this, _1), false)
  {
    as_.start();
    pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/test/objectposes", 10);
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
    result_.objects_ids.clear();
    result_.x_plane = 0;
    result_.y_plane = 0;
    result_.z_plane = 0;
    result_.width_plane = 0;
    result_.height_plane = 0;
    pose_pub_msg_.poses.clear();

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM("Action Server Detect3D - Preempted");
      as_.setPreempted(); // Set the action state to preempted
      return;
    }

    boost::shared_ptr<sensor_msgs::PointCloud2 const> input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/points", nh_);
    Detect3D::cloudCB(input_cloud);
    as_.setSucceeded(result_);
    pose_pub_.publish(pose_pub_msg_);
  }

  /** \brief Given the parameters of the object add it to the planning scene. */
  void addObject(const int id, const ObjectParams& object_found, const PlaneParams &table_params)
  {
    // Adding Object to Planning Scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "map";
    collision_object.id = "object-" + std::to_string(id);

    geometry_msgs::PoseStamped object_pose;
    object_pose.header.stamp = ros::Time::now();
    object_pose.header.frame_id = "camera_depth_frame";
    object_pose.pose.position = object_found.center.position;
    object_pose.pose.orientation = object_found.center.orientation;

    geometry_msgs::TransformStamped tf_to_map;
    tf_to_map = buffer_.lookupTransform("map", object_pose.header.frame_id, ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(object_pose, object_pose, tf_to_map);

    // Add object only if is above table.
    if (object_pose.pose.position.z < table_params.world_z) {
      return;
    }

    collision_object.meshes.push_back(*object_found.mesh);
    collision_object.mesh_poses.push_back(object_pose.pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);

    result_.objects_found++;
    pose_pub_msg_.header = object_pose.header;
    pose_pub_msg_.poses.push_back(object_pose.pose);
    result_.objects_poses.push_back(object_pose);
    result_.objects_names.push_back(collision_object.id);
    result_.objects_ids.push_back(object_found.label);
  }

  /** \brief Given the parameters of the plane add it to the planning scene. */
  bool addPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients_plane, bool addToMoveit, PlaneParams &plane_params)
  {
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
    
    geometry_msgs::TransformStamped tf_to_map;
    tf_to_map = buffer_.lookupTransform("map", plane_pose.header.frame_id, ros::Time(0), ros::Duration(1.0) );
    tf2::doTransform(plane_pose, plane_pose, tf_to_map);

    plane_pose.pose.position.z = plane_params.world_z - 0.04;
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
    if (addToMoveit) {
      collision_object.primitives.push_back(solid_primitive);
      collision_object.primitive_poses.push_back(plane_pose.pose);
      collision_object.operation = collision_object.ADD;
      planning_scene_interface_.applyCollisionObject(collision_object);
    }

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

  /** \brief Given pcl Polygon Mesh convert it to ros Mesh.
      @param polygon_mesh_ptr - Polygon Mesh Pointer. */
  bool convertPolygonMeshToRosMesh(const pcl::PolygonMesh::Ptr polygon_mesh_ptr, shape_msgs::Mesh::Ptr ros_mesh_ptr) {
    ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh started.");

    pcl_msgs::PolygonMesh pcl_msg_mesh;

    pcl_conversions::fromPCL(*polygon_mesh_ptr, pcl_msg_mesh);

    sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

    size_t size = pcd_modifier.size();

    ros_mesh_ptr->vertices.resize(size);

    ROS_INFO_STREAM("polys: " << pcl_msg_mesh.polygons.size()
                              << " vertices: " << pcd_modifier.size());

    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

    for (size_t i = 0u; i < size; i++, ++pt_iter) {
      ros_mesh_ptr->vertices[i].x = pt_iter[0];
      ros_mesh_ptr->vertices[i].y = pt_iter[1];
      ros_mesh_ptr->vertices[i].z = pt_iter[2];
    }

    ROS_INFO_STREAM("Updated vertices");

    ros_mesh_ptr->triangles.resize(polygon_mesh_ptr->polygons.size());

    for (size_t i = 0u; i < polygon_mesh_ptr->polygons.size(); ++i) {
      if (polygon_mesh_ptr->polygons[i].vertices.size() < 3u) {
        ROS_WARN_STREAM("Not enough points in polygon. Ignoring it.");
        continue;
      }

      for (size_t j = 0u; j < 3u; ++j) {
        ros_mesh_ptr->triangles[i].vertex_indices[j] =
            polygon_mesh_ptr->polygons[i].vertices[j];
      }
    }
    ROS_INFO("Conversion from PCL PolygonMesh to ROS Mesh ended.");
    return true;
  }

  /** \brief Given the pointcloud containing just the object, reconstruct a mesh from it.
      @param cloud - point cloud containing just the object. */
  void reconstructMesh(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, shape_msgs::Mesh::Ptr &mesh, int id)
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
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
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
  void extractObjectDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ObjectParams& object_found, int id)
  {
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);

    // Store the object centroid.
    object_found.center.position.x = centroid.x;
    object_found.center.position.y = centroid.y;
    object_found.center.position.z = centroid.z;
    object_found.center.orientation.x = 0.0;
    object_found.center.orientation.y = 0.0;
    object_found.center.orientation.z = 0.0;
    object_found.center.orientation.w = 1.0;

    // Change point cloud origin to object centroid and save it.
    for (auto &point : cloud->points)
    {
      point.x = point.x - centroid.x;
      point.y = point.y - centroid.y;
      point.z = point.z - centroid.z;
    }
    object_found.cluster = cloud;
    
    // Store mesh.
    reconstructMesh(cloud, object_found.mesh, id);
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
    plane_found.world_z  = plane_pose.pose.position.z;
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
  bool removeBiggestPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane, PlaneParams &plane_params)
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

    isTheTable = addPlane(planeCloud, coefficients_plane, false, plane_params);
    if (isTheTable) {
      pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
      pcl::io::savePCDFile("pcl_no_table.pcd", *cloud);
    }
    return isTheTable;
  }
  
  /** \brief Calculate Distance between two points. */
  float getDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
  }

  /** \brief Bind an object with a detection Distance between two points. */
  void bindDetections(std::vector<ObjectParams> &objects) {
    // Get the best of three different detections.
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections2 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
    boost::shared_ptr<object_detector::objectDetectionArray const> input_detections3 = ros::topic::waitForMessage<object_detector::objectDetectionArray>("/detections", nh_);
    input_detections = input_detections->detections.size() < input_detections2->detections.size() ? input_detections2 : input_detections;
    input_detections = input_detections->detections.size() < input_detections3->detections.size() ? input_detections3 : input_detections;

    int n_detections = input_detections->detections.size();
    
    for(int i=0;i<n_detections;i++) {
      float min_distance = std::numeric_limits<float>::max();
      int min_index = -1;
      for(int j=0;j<objects.size();j++) {
        float curr_distance = getDistance(objects[j].center.position, input_detections->detections[i].point3D);
        if (curr_distance < min_distance) {
          min_distance = curr_distance;
          min_index = j;
        }
      }
      if (min_index != -1) {
        objects[min_index].label = input_detections->detections[i].label;
        ROS_INFO_STREAM("Detection " << i << " binded with object " << min_index << " -> Min Distance: " << min_distance);
      }
    }

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
    PlaneParams table_params;
    while(!tableRemoved && !cloud->points.empty()) {
      tableRemoved = removeBiggestPlane(cloud, inliers_plane, table_params);
      extractNormals(cloud_normals, inliers_plane);
    }

    if (cloud->points.empty()) {
      return;
    }

    /* Extract all objects from PointCloud using Clustering. */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    getClusters(cloud, clusters);
    int clustersFound = clusters.size();

    std::vector<ObjectParams> objects(clustersFound);
    for(int i=0;i < clustersFound; i++) {
      objects[i].mesh.reset(new shape_msgs::Mesh);
      pcl::io::savePCDFile("pcl_object_"+std::to_string(i)+".pcd", *clusters[i]);
      extractObjectDetails(clusters[i], objects[i], i);
    }

    bindDetections(objects);

    for(int i=0;i < clustersFound; i++) {
      addObject(i, objects[i], table_params);
    }
  }

  /** \brief Find all clusters in a pointcloud.*/
  void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.04); // 4cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(30000);
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
