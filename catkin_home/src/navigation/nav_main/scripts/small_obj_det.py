#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_multiply
from nav_msgs.msg import Odometry

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.robot_pose = None
        self.object_pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/zed2i/zed_node/point_cloud/cloud_registered', PointCloud2, self.callback, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.robot_pose_callback, queue_size=1)

    def callback(self, point_cloud):
        # Only process point cloud if robot pose is available
        if self.robot_pose is None:
            return

        # Create generator to get x,y,z coordinates from point cloud
        gen = pc2.read_points(point_cloud, skip_nans=True, field_names=("x", "y", "z"))

        # Iterate over point cloud to find small objects
        for point in gen:
            if point[2] <= 0.17:  # Check if point is less than 17cm from the camera
                # Convert point coordinates to robot frame
                point_robot_frame = self.transform_point(point)

                # Create PoseStamped message for object pose and publish
                object_pose = PoseStamped()
                object_pose.header.stamp = rospy.Time.now()
                object_pose.header.frame_id = 'robot_frame'
                object_pose.pose.position = Point(point_robot_frame[0], point_robot_frame[1], point_robot_frame[2])
                object_pose.pose.orientation.w = 1.0  # No rotation
                self.object_pose_pub.publish(object_pose)

    def transform_point(self, point):
        # Transform point from camera frame to robot frame
        x, y, z = point
        # Create quaternion from robot pose orientation
        q = [self.robot_pose.pose.pose.orientation.x,
             self.robot_pose.pose.pose.orientation.y,
             self.robot_pose.pose.pose.orientation.z,
             self.robot_pose.pose.pose.orientation.w]
        # Create inverse quaternion to transform point to robot frame
        q_inv = [q[0], -q[1], -q[2], -q[3]]
        # Multiply quaternion by point and then by inverse quaternion to get point in robot frame
        point_robot_frame = quaternion_multiply(quaternion_multiply(q, [x, y, z, 0]), q_inv)[:3]
        return point_robot_frame

    def robot_pose_callback(self, odom):
        # Extract robot position and orientation from Odometry message
        self.robot_pose = odom.pose

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
