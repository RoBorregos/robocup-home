#!/usr/bin/env python3
import mediapipe as mp
from time import sleep
from typing import Tuple
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
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


class FollowPerson:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            '/coordinate_move_base', MoveBaseGoal, self.goal_callback)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.newgoal=MoveBaseGoal()
        self.newgoal.target_pose.pose.position.x=-1
        self.old_x=-1
    def goal_callback(self,msg):
        self.newgoal=msg

    def move_goal(self):
        #if coordinate is new, move
        if self.old_x != self.newgoal.target_pose.pose.position.x:
            self.old_x = self.newgoal.target_pose.pose.position.x
            rospy.logwarn(self.newgoal)
            self.client.send_goal(self.newgoal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                rospy.logwarn(self.client.get_result())



if __name__ == '__main__':
    rospy.init_node('follow_humano', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    follow = FollowPerson()
    try:
        while not rospy.is_shutdown():
            follow.move_goal()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard interrupt detected, stopping listener")