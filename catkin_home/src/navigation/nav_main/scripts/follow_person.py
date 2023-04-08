#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped

class ObjectPoseNEDConverter:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_pose_ned_converter')

        # Define NED reference frame origin (change these values to match your use case)
        self.ned_origin = np.array([0.0, 0.0, 0.0])

        # Define transformation matrix (change these values to match your use case)
        self.rot_matrix = np.array([[1.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0]])
        self.trans_vector = np.array([0.0, 0.0, 0.0])
        self.quat = np.array([0.0, 0.0, 0.0, 0.0])

        # Subscribe to odometry messages
        self.odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback)

        # Subscribe to object detection messages
        self.obj_sub = rospy.Subscriber('/object_detection', PointStamped, self.obj_callback)

        self.pub_follow = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        #Distance to human
        self.human_distance = np.array([0,0,0.0])
        # Initialize transformation matrix to identity matrix
        self.transform_matrix = np.identity(4)

    def odom_callback(self, msg):
        # Extract the position and orientation from the odometry message
        self.ned_origin = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        # Calculate the rotation matrix from the quaternion
        rot_matrix = self.quaternion_to_matrix(self.quat)

        # Combine the rotation matrix and translation vector into a transformation matrix
        transform_matrix = np.vstack((np.hstack((rot_matrix, self.trans_vector[:, np.newaxis])), np.array([0, 0, 0, 1])))

        # Store the transformation matrix
        self.transform_matrix = transform_matrix

    def obj_callback(self, msg):
        # Extract the position of the object in the local reference frame
        obj_pos_local = np.array([msg.point.x, msg.point.y, msg.point.z])

        # Apply the transformation to obtain the object position in NED coordinates
        obj_pos_ned = np.dot(self.transform_matrix, np.hstack((obj_pos_local, 1)))[:3] + self.ned_origin + self.human_distance

        # Print the object position in NED coordinates
        rospy.logwarn('Object position in NED coordinates: {}'.format(obj_pos_ned))
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position.x = obj_pos_ned[0]
        pose.pose.position.y = obj_pos_ned[1]
        pose.pose.position.z = obj_pos_ned[2]
        pose.pose.orientation.x = self.quat[0]
        pose.pose.orientation.y = self.quat[1]
        pose.pose.orientation.z = self.quat[2]
        pose.pose.orientation.w = self.quat[3]
        self.pub_follow.publish(pose)

    def quaternion_to_matrix(self, quat):
        # Convert a quaternion to a rotation matrix
        x, y, z, w = quat
        return np.array([[1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                         [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
                         [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

if __name__ == '__main__':
    converter = ObjectPoseNEDConverter()
    rospy.spin()
