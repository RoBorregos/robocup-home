#!/usr/bin/env python3

import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

# ZED camera parameters
CAM_HEIGHT = 0.5  # m
CAM_ANGLE = 45.0  # deg

# Minimum object height
MIN_HEIGHT = 0.17  # m

class ObjectDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_detector', anonymous=True)

        # Subscribe to point cloud and odom topics
        rospy.Subscriber('/zed2i/zed_node/point_cloud/ds_cloud_registered', PointCloud2, self.callback_pointcloud)
        rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # Publisher for object position
        self.pub_obj_pos = rospy.Publisher('/object_position', PointStamped, queue_size=10)

        # Initialize transformation matrix from camera frame to robot frame
        self.T_cam_to_robot = np.array([
            [0.0, -1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, CAM_HEIGHT],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Initialize rotation matrix for camera angle
        theta = np.deg2rad(CAM_ANGLE)
        self.R_cam_angle = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, np.cos(theta), -np.sin(theta), 0.0],
            [0.0, np.sin(theta), np.cos(theta), 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Initialize robot pose
        self.robot_pos = None

    def callback_pointcloud(self, msg):
        # Convert point cloud message to numpy array
        cloud = np.array(list(pc2.read_points(msg, skip_nans=True)))

        # Transform point cloud to robot frame
        cloud_robot = self.T_cam_to_robot @ self.R_cam_angle @ cloud.T
        cloud_robot = cloud_robot.T

        # Filter objects with height less than MIN_HEIGHT
        mask = cloud_robot[:, 2] < MIN_HEIGHT
        cloud_robot = cloud_robot[mask]

        # Compute object position in robot frame
        if len(cloud_robot) > 0:
            obj_pos = np.mean(cloud_robot[:, :3], axis=0)

            # Publish object position
            if self.robot_pos is not None:
                obj_pos_robot = self.robot_pos @ np.append(obj_pos, 1.0)
                obj_pos_robot = obj_pos_robot[:3] / obj_pos_robot[3]
                obj_pos_robot_msg = PointStamped()
                obj_pos_robot_msg.header.stamp = rospy.Time.now()
                obj_pos_robot_msg.header.frame_id = 'odom'
                obj_pos_robot_msg.point.x = obj_pos_robot[0]
                obj_pos_robot_msg.point.y
