#!/usr/bin/env python3
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
            "living room": {
                "receptionist": [
                    1.373014211654663,
                    1.1684893369674683,
                    -0.5000007152557373,
                    -1.1124803478423928e-07,
                    4.741806947095029e-10,
                    -0.9319508671760559,
                    0.36258524656295776
                ],
                "turning couch": [
                    2.509934186935425,
                    0.2755257487297058,
                    -0.4999988079071045,
                    -4.951883170178917e-08,
                    9.025715730359707e-10,
                    -0.4073343276977539,
                    0.9132794141769409
                ],
                "turning entry couches": [
                    2.509934186935425,
                    0.2755257487297058,
                    -0.4999988079071045,
                    -1.0058943189505953e-07,
                    2.5082991239600005e-10,
                    -0.8354406952857971,
                    0.5495811700820923
                ]
            }
        }
    def move_goal(self):
        #if coordinate is new, move
        for i in self.mydict:
            for j in self.mydict[i]:
                self.newgoal.target_pose.header.frame_id = "map"
                self.newgoal.target_pose.header.stamp = rospy.Time.now()
                self.newgoal.target_pose.pose.position.x = self.mydict[i][j][0]
                self.newgoal.target_pose.pose.position.y = self.mydict[i][j][1]
                self.newgoal.target_pose.pose.position.z = self.mydict[i][j][2]
                self.newgoal.target_pose.pose.orientation.x = self.mydict[i][j][3]
                self.newgoal.target_pose.pose.orientation.y = self.mydict[i][j][4]
                self.newgoal.target_pose.pose.orientation.z = self.mydict[i][j][5]
                self.newgoal.target_pose.pose.orientation.w = self.mydict[i][j][6]  
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
    follow = TestCoordinates()
    try:
        while not rospy.is_shutdown():
            follow.move_goal()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard interrupt detected, stopping listener")