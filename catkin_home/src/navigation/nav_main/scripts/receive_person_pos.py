#!/usr/bin/env python3
import rospy
import tf2_ros

from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

import numpy as np
import math
import actionlib


class HumanPositionGetter:
    """
    Receives a Point Stamped (human back position) from the topic published by jetson xavier in the odom transform.
    """

    def __init__(self) -> None:
        self.rate = rospy.Rate(10)
        self.follow_person = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin_x = -1
        self.map_origin_y = -1

        self.current_person_pos = (-1, -1)
        self.last_person_pos = (-1, -1)
        self.map_data = []

        self.modified_map_publisher = rospy.Publisher(
            "/map", OccupancyGrid, queue_size=10
        )
        self.map_subscriber = rospy.Subscriber(
            "/map_original", OccupancyGrid, self.map_callback
        )

        rospy.loginfo("Initializing")
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("Initializing")

        self.pose_subscriber = rospy.Subscriber(
            "/person_pose_odom", PointStamped, self.pose_callback
        )
        self.follow_person_subscriber = rospy.Subscriber(
            "/follow_person", Bool, self.follow_person_callback
        )
        self.odom_subscriber = rospy.Subscriber(
            # Robot_pose when running AMCL and run slam_out_pose when running Hector_Slam
            "/slam_out_pose",
            PoseStamped,
            self.odom_callback,
        )
        self.goal_sub = rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self.goal_callback
        )

        self.person_pose_map_pub = rospy.Publisher(
            "/person_pose_map", PoseStamped, queue_size=10
        )

        rospy.loginfo("Topic Ready")

    def get_area(self, width, center_row, center_col):
        print(
            center_row - width // 2,
            center_row + width // 2,
            center_col - width // 2,
            center_col + width // 2,
        )
        return (
            int(max(center_row - width // 2, 0)),
            int(min(center_row + width // 2, self.map_height)),
            int(max(center_col - width // 2, 0)),
            int(min(center_col + width // 2, self.map_width)),
        )

    def set_area_to_value(
        self, map: OccupancyGrid, value, start_row, end_row, start_col, end_col
    ):
        print(start_row, end_row, start_col, end_col)
        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                self.modify_map(map, row, col, value)

    def modify_map(self, map: OccupancyGrid, row, col, value):
        index = self.map_width * row + col
        
        self.map_data[index] = value

    def publish_map(self, map: OccupancyGrid):
        map.data = tuple(self.map_data)
        self.modified_map_publisher.publish(map)

    def map_callback(self, map: OccupancyGrid):
        self.map_data = list(map.data)
        self.map_width = map.info.width
        self.map_height = map.info.height
        self.map_resolution = map.info.resolution
        self.map_origin_x = map.info.origin.position.x
        self.map_origin_y = map.info.origin.position.y

        # if not self.follow_person:
        #     self.publish_map(map)
        #     return

        if self.last_person_pos != (-1, -1):
            print("removing last person pos")
            self.set_area_to_value(map, -1, *self.get_area(15, *self.last_person_pos))

        self.last_person_pos = (-1, -1)

        if self.current_person_pos != (-1, -1):
            print("adding current person pos")
            self.set_area_to_value(
                map, 100, *self.get_area(15, *self.current_person_pos)
            )

        self.publish_map(map)

    def get_position_cell(self, x, y):
        y -= self.map_origin_y
        x -= self.map_origin_x

        row = y // self.map_resolution
        col = x // self.map_resolution

        print(f"get position cell: {row, col}")

        return row, col

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
        print(f"Received: {follow_person}")
        self.follow_person = follow_person.data

    def odom_callback(self, robot_pose: PoseStamped):
        robot_pose = robot_pose.pose
        self.robot_position = np.array(
            [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]
        )
        self.robot_orientation = np.array(
            [
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
            ]
        )

    # For thread on sending new goal once it completes the last one
    def goal_callback(self, data):
        self.goal_status = data.status_list

    def pose_callback(self, person_pose_odom: PointStamped):
        print(
            f"pose callback {person_pose_odom.point.x}, {person_pose_odom.point.y}, {person_pose_odom.point.z}"
        )
        if not self.follow_person:
            return

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_footprint"

        pose.pose.position.x = person_pose_odom.point.x
        pose.pose.position.y = person_pose_odom.point.y
        pose.pose.position.z = 0

        distance_to_person = np.sqrt(
            (pose.pose.position.x) ** 2 + (pose.pose.position.y) ** 2
        )
        if distance_to_person < 0.5 or distance_to_person > 2.5:
            print("Person too close or too far")
            return

        safe_distance_multiplier = 0.6
        pose.pose.position.x *= safe_distance_multiplier
        pose.pose.position.y *= safe_distance_multiplier

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
            person_pose_map = self.tf_buffer.transform(pose, "map", rospy.Duration(1))
        except Exception as e:
            print(e)
            rospy.logerr("Failed to transform object pose from body_frame to map_frame")
            self.rate.sleep()
            return
            # rospy.signal_shutdown("Transformation failed")

        # print(person_pose_map)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = person_pose_map.pose.position.x
        pose.pose.position.y = person_pose_map.pose.position.y
        pose.pose.position.z = 0

        # update person positions for map modifications
        self.last_person_pos = self.current_person_pos
        self.current_person_pos = self.get_position_cell(
            pose.pose.position.x, pose.pose.position.y
        )

        self.angle = math.atan2(
            (person_pose_map.pose.position.y - self.robot_position[1]),
            (person_pose_map.pose.position.x - self.robot_position[0]),
        )

        quat = self.get_quaternion_from_euler(0, 0, self.angle)

        print(f"Robot angle: {self.robot_orientation[2]}    Goal Angle: {self.angle}")

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.person_pose_map_pub.publish(pose)

        if self.follow_person:
            # if (
            #     self.goal_status == 3 or self.goal_status == -1 or self.goal_status == 4
            # ) and (pose.pose.position.x - self.ned_origin[0]) <= 4:
            print("sending goal")
            goal = MoveBaseGoal()
            goal.target_pose = pose
            self.move_client.send_goal(goal)
        # self.move_client.wait_for_result()

        # Publish nav goal
        # self.pub_follow.publish(pose)


if __name__ == "__main__":
    rospy.init_node("human_pose_getter", anonymous=True)

    humanGetter = HumanPositionGetter()
    rospy.spin()
