#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import math
from tf.transformations import quaternion_from_euler
import tf

class FollowPoint:
    def __init__(self):
        rospy.init_node('FollowPoint', anonymous=True)

        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.clicked_point_subscriber = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)

        self.goal_publisher = rospy.Publisher('/clicked_point_goal', PoseStamped, queue_size=10)

    def clicked_point_callback(self, clicked_point):
        rospy.loginfo('Received clicked point: {}'.format(clicked_point))
        # Transform clicked point to base_link frame
        clicked_point_transformed = self.transform_to_base_link_frame(clicked_point)
        clicked_point_transformed.point.z = 0
        # Calculate goal position and orientation
        goal_position = self.calculate_goal_position(clicked_point_transformed)
        goal_orientation = self.calculate_goal_orientation(clicked_point_transformed)

        # Send move_base goal
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'base_link'
        move_base_goal.target_pose.pose.position = goal_position
        move_base_goal.target_pose.pose.orientation = goal_orientation
        rospy.loginfo('Sending move_base goal: {}'.format(move_base_goal))
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()
        # print("goal_position: ", goal_position)
        self.goal_publisher.publish(move_base_goal.target_pose)

    def transform_to_base_link_frame(self, clicked_point):
        transform_listener = tf.TransformListener()
        transform_listener.waitForTransform('base_link', clicked_point.header.frame_id, rospy.Time(0), rospy.Duration(10.0))
        clicked_point_transformed = transform_listener.transformPoint('base_link', clicked_point)
        return clicked_point_transformed

    def calculate_goal_position(self, clicked_point_transformed):
        # Calculate a vector from the origin to the clicked point
        origin = PointStamped()
        origin.header.frame_id = 'base_link'
        origin.point.x = 0.0
        origin.point.y = 0.0
        origin.point.z = 0.0
        vector_x = clicked_point_transformed.point.x - origin.point.x
        vector_y = clicked_point_transformed.point.y - origin.point.y
        vector_z = clicked_point_transformed.point.z - origin.point.z

        # Calculate the magnitude of the vector and normalize it
        SAFE_DISTANCE = 0.4
        magnitude = (math.sqrt(vector_x ** 2 + vector_y ** 2 + vector_z ** 2))
        vector_x /= magnitude
        vector_y /= magnitude
        vector_z /= magnitude
        magnitude = max(0, magnitude - SAFE_DISTANCE)
        vector_x *= magnitude
        vector_y *= magnitude
        vector_z *= magnitude
        
        # Calculate the goal position
        goal_position = PoseStamped()
        goal_position.header.frame_id = 'base_link'
        goal_position.pose.position.x = origin.point.x + vector_x
        goal_position.pose.position.y = origin.point.y + vector_y
        goal_position.pose.position.z = origin.point.z + vector_z
        return goal_position.pose.position

    def calculate_goal_orientation(self, clicked_point_transformed):
        # Calculate a vector from the origin to the clicked point
        origin = PointStamped()
        origin.header.frame_id = 'base_link'
        origin.point.x = 0.0
        origin.point.y = 0.0
        origin.point.z = 0.0
        vector_x = clicked_point_transformed.point.x - origin.point.x
        vector_y = clicked_point_transformed.point.y - origin.point.y
        vector_z = clicked_point_transformed.point.z - origin.point.z

        # Calculate the yaw angle of the vector
        yaw = math.atan2(vector_y, vector_x)

        # Calculate the goal orientation
        goal_orientation = PoseStamped()
        goal_orientation.header.frame_id = 'base_link'
        goal_orientation.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, yaw))
        return goal_orientation.pose.orientation


if __name__ == '__main__':
    clicked_point_subscriber = FollowPoint()
    rospy.spin()