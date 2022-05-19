#! /usr/bin/env python3

import json
import math
import tf

import pathlib
import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actions.msg
from actions.msg import navServAction, navServGoal, navServResult

BASE_PATH = str(pathlib.Path(__file__).parent) + '/../../../../'

placesPoses = {
 "KITCHEN": Pose(Point(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
 "COUCH": Pose(Point(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
 "BATHROOM": Pose(Point(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
 "CLOSET": Pose(Point(x=0.0, y=0.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
}

class navigationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        rospy.loginfo("Waiting for Robot...")
        odom_msg = rospy.wait_for_message("odom", Odometry)
        rospy.loginfo("Robot Launched...")

        rospy.loginfo("Waiting for MoveBase AS...")
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        # Localization
        rospy.wait_for_service('/global_localization')
        global_localization = rospy.ServiceProxy('/global_localization', Empty)
        global_localization()
        self.send_relative_goal()
        self.send_relative_goal()

        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        target = goal.target_location
        rospy.loginfo("Robot Moving Towards " + target)
        # self.send_goal(placesPoses[target])
        self._as.set_succeeded(navServResult(result=True))

    def send_goal(self, target_pose):
        goal = MoveBaseGoal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = target_pose
        goal.target_pose = pose_stamped
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
    
    def send_relative_goal(self, target_pose):
        goal = MoveBaseGoal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(180.0))
        goal.target_pose = pose_stamped
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()