#!/usr/bin/env python3

"""
This node receives the position published by the human_position_publisher node 
and transforms it to the map frame. It then publishes the position to the
move_base action server to navigate to the person.

This node runs on the jetson nano.
"""

import rospy
from geometry_msgs.msg import PointStamped, Pose
from tf2_geometry_msgs import PoseStamped
import tf2_ros
from std_msgs.msg import Bool
import numpy as np
import math
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class HumanPositionGetter:
    """
    Receives a Point Stamped (human back position) from the topic published by jetson xavier in the odom transform.
    """

    def __init__(self) -> None:
        self.rate = rospy.Rate(10)
        self.follow_person = True

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.tf_listener.wait_for_transform("map", "base_footprint", rospy.Time(), rospy.Duration(1))

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.pose_subscriber = rospy.Subscriber(
            "/person_pose_odom", PointStamped, self.pose_callback
        )
        self.follow_person_subscriber = rospy.Subscriber(
            "/follow_person", Bool, self.follow_person_callback
        )
        self.odom_subscriber = rospy.Subscriber(
            "/robot_pose", Pose, self.odom_callback
        )
        self.goal_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.goal_callback
        )
        self.person_pose_map_pub = rospy.Publisher(
            "/person_pose_map", PoseStamped, queue_size=10
        )


    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def follow_person_callback(self, follow_person: Bool):
        print("follow person")
        self.follow_person = follow_person

    def odom_callback(self, robot_pose: Pose):
        self.robot_position = np.array(
            [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]
        )

    # For thread on sending new goal once it completes the last one
    def goal_callback(self, data):
        self.goal_status = data.status_list

    def pose_callback(self, person_pose_odom: PointStamped):
        print(f"pose callback {person_pose_odom.point.x}, {person_pose_odom.point.y}, {person_pose_odom.point.z}")
        if not self.follow_person:
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_footprint"

        pose.pose.position.x = person_pose_odom.point.x
        pose.pose.position.y = person_pose_odom.point.y
        pose.pose.position.z = 0

        try:
            self.tf_buffer.lookup_transform("map", "base_footprint", rospy.Time())
            # self.tf_buffer.wait_for_transform("map", "base_footprint", rospy.Time(), rospy.Duration(1))
            # self.tf_buffer.can_transform("map", "base_footprint", rospy.Time())
        except Exception as e:
            print(e)
            rospy.logerr("error on transformation lookup")
            self.rate.sleep()
            return
            # rospy.signal_shutdown("transformation not available")

        try:
            person_pose_map = self.tf_buffer.transform(
                pose, "map", rospy.Duration(1)
            )
        except Exception as e:
            print(e)
            rospy.logerr("Failed to transform object pose from body_frame to map_frame")
            self.rate.sleep()
            return
            # rospy.signal_shutdown("Transformation failed")
        
        print(person_pose_map)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = person_pose_map.pose.position.x
        pose.pose.position.y = person_pose_map.pose.position.y
        pose.pose.position.z = 0

        self.angle = math.atan(
            (person_pose_map.pose.position.y - self.robot_position[1])
            / (person_pose_map.pose.position.x - self.robot_position[0])
        )
        quat = self.get_quaternion_from_euler(0, 0, self.angle)

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.person_pose_map_pub.publish(pose)

        # if (
        #     self.goal_status == 3 or self.goal_status == -1 or self.goal_status == 4
        # ) and (pose.pose.position.x - self.ned_origin[0]) <= 4:
        # print("sending goal")
        # goal = MoveBaseGoal()
        # goal.target_pose = pose
        # self.move_client.send_goal(goal)
        # self.move_client.wait_for_result()

            # Publish nav goal
        # self.pub_follow.publish(pose)

if __name__ == '__main__': 
    rospy.init_node('human_pose_getter', anonymous=True)
    
    humanGetter = HumanPositionGetter()
    rospy.spin()