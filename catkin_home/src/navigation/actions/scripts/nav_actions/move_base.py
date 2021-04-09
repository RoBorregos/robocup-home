#! /usr/bin/env python
from __future__ import print_function

from math import degrees, radians

import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actions.msg

"""
Valid States:
    Pending = 0
    Active = 1
    Preempted = 2
    Succeeded = 3
    Aborted = 4
    Rejected = 5
    Preempting = 6
    Recalling = 7
    Recalled = 8
    Lost = 9

Reference: docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
"""

class MoveBase(object):
    _goal = MoveBaseGoal()
    _moveBaseStatus = ["PENDING","ACTIVE", "SUCCEDED","ABORTED"]

    def __init__(self):
        self._ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while (not self._ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
    
    def setGoal(self,goal_pose_given):
        #set up the frame parameters
        self._goal.target_pose.header.frame_id = "map"
        self._goal.target_pose.header.stamp = rospy.Time.now()
        # set up goal pose and orientation
        self._goal.target_pose.pose.position =  Point(goal_pose_given[0],goal_pose_given[1],0)
        self._goal.target_pose.pose.orientation.x = goal_pose_given[2]
        self._goal.target_pose.pose.orientation.y = goal_pose_given[3]
        self._goal.target_pose.pose.orientation.z = goal_pose_given[4]
        self._goal.target_pose.pose.orientation.w = goal_pose_given[5]

    def getGoal(self):
        return self._goal
    
    def cancelGoal(self):
        self._ac.cancel_all_goals()
        rospy.loginfo('Goal Cancelled')
        return True

    def sendGoalToNavStack(self):
        self._ac.send_goal(self.getGoal())
        self._ac.wait_for_result(rospy.Duration(120))
        return (self._ac.get_state() ==  GoalStatus.SUCCEEDED)

    def getMoveBaseStatus(self, data):
        if not len(data.status_list): return self._moveBaseStatus[0]

        self.status = data.status_list[0].status

        if self.status < 2:
            return self._moveBaseStatus[self.status]
        elif self.status == 2 or self.status > 4:
            return self._moveBaseStatus[3]
        else:
            return self._moveBaseStatus[self.status - 1]

if __name__ == '__main__':
    moveBase = MoveBase()
