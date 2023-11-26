#!/usr/bin/env python3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from time import sleep
from typing import Tuple
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped, PointStamped, PoseWithCovarianceStamped
import math
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseArray, Pose
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import tf2_geometry_msgs  # Import the tf2_geometry_msgs library
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
FLT_EPSILON = sys.float_info.epsilon


class TestCoordinates:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.newgoal=MoveBaseGoal()
        self.mydict={
            "InitialTest": {
                "init test": [
                    2.5315836906433105,
                    -0.24596719443798065,
                    0.0,
                    0.0,
                    0.0,
                    0.6757392883300781,
                    0.7371407151222229
                ],
                "exit": [
                    1.0586,
                    1.35,
                    0.0,
                    0.0,
                    0.0,
                    0.71006,
                    0.7041415162845805
                ]
            }
        }

    def move_goal(self):
        #if coordinate is new, move
        # for i in self.mydict:
        #     for j in self.mydict[i]:
        self.newgoal.target_pose.header.frame_id = "map"
        self.newgoal.target_pose.header.stamp = rospy.Time.now()
        self.newgoal.target_pose.pose.position.x = self.mydict["InitialTest"]["init test"][0]
        self.newgoal.target_pose.pose.position.y = self.mydict["InitialTest"]["init test"][1]
        self.newgoal.target_pose.pose.position.z = self.mydict["InitialTest"]["init test"][2]
        self.newgoal.target_pose.pose.orientation.x = self.mydict["InitialTest"]["init test"][3]
        self.newgoal.target_pose.pose.orientation.y = self.mydict["InitialTest"]["init test"][4]
        self.newgoal.target_pose.pose.orientation.z = self.mydict["InitialTest"]["init test"][5]
        self.newgoal.target_pose.pose.orientation.w = self.mydict["InitialTest"]["init test"][6]  
        self.client.send_goal(self.newgoal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.logwarn(self.client.get_result())
        rospy.sleep(5)    
        self.newgoal.target_pose.header.frame_id = "map"
        self.newgoal.target_pose.header.stamp = rospy.Time.now()
        self.newgoal.target_pose.pose.position.x = self.mydict["InitialTest"]["exit"][0]
        self.newgoal.target_pose.pose.position.y = self.mydict["InitialTest"]["exit"][1]
        self.newgoal.target_pose.pose.position.z = self.mydict["InitialTest"]["exit"][2]
        self.newgoal.target_pose.pose.orientation.x = self.mydict["InitialTest"]["exit"][3]
        self.newgoal.target_pose.pose.orientation.y = self.mydict["InitialTest"]["exit"][4]
        self.newgoal.target_pose.pose.orientation.z = self.mydict["InitialTest"]["exit"][5]
        self.newgoal.target_pose.pose.orientation.w = self.mydict["InitialTest"]["exit"][6]  
        self.client.send_goal(self.newgoal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.logwarn(self.client.get_result())
                             
            

class DoorDetector:
    def __init__(self):
        self.last_distance = None
        self.door_opened = False

        self.firstIteration = True
        # Subscribe to the LiDAR data topic
        # rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/scan_filtered', LaserScan, self.lidar_callback)

        # Create a publisher for the movement command topic
        self.cmd_pub = rospy.Publisher('/move_command', String, queue_size=10)

    def lidar_callback(self, msg):
        # Filter the LiDAR data to only include points near the door
        door_angle = 1.6  # Replace with the angle of the door in the LiDAR data (rad) 1.6 is straight of our robot
        #door_distance = 0.45  # Replace with the distance of the door in the LiDAR data (m)
        door_idx = int(door_angle / msg.angle_increment)
        min_idx = max(0, door_idx - 10)
        max_idx = min(len(msg.ranges), door_idx + 10)
        door_ranges = msg.ranges[min_idx:max_idx]
        # Calculate the median distance to the door
        door_distance = sorted(door_ranges)[len(door_ranges)//2]
        rospy.loginfo("Door: %f", door_distance)
        # Check if the distance has changed significantly since the last scan
        if self.last_distance is not None and abs(door_distance - self.last_distance) > 0.30:   #> #.## (m) how much difference from door_distance
            self.door_opened = True
        self.last_distance = door_distance

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.door_opened:
                # Send a message to the robot's control node to start moving
                # cmd_msg = String()
                # cmd_msg.data = "start_moving"
                # self.cmd_pub.publish(cmd_msg)
                return True
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('follow_humano', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    follow = TestCoordinates()
    detector = DoorDetector()
    try:
        if not rospy.is_shutdown():
            if detector.run():
                follow.move_goal()
                rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard interrupt detected, stopping listener")