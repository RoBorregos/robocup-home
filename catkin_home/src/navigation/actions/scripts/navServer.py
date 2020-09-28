#! /usr/bin/env python
from __future__ import print_function

import csv
import json
from math import degrees, radians

import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actions.msg


""" 
Valid states:
    PENDING
    ACTIVE
    SUCCEEDED
    ABORTED
Valid places:
    kitchen
    restroom
    bedroom
"""


class navigationServer(object):
    # Create messages that are used to publish feedback/result
    _feedback = actions.msg.navServFeedback()
    _result = actions.msg.navServResult()
    goals = {}

    def __init__(self, name):
        self._action_name = name
        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # declare the coordinates of interest
        a = ['entrance', 'bedroom', 'kitchen', 'restroom', 'dinning_room']
        place = 0
        self.goals = {}
        with open('src/navigation/actions/data/goals.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            goal = [float(i) for i in row]
            self.goals[a[place]] = goal
            place=(place+1)%len(a)
        rospy.loginfo(self.goals)

    def validateGoal(self, goal):
        if(type(goal) != str):
            return false
        rospy.loginfo("Place received: %s", goal)
        if goal in self.goals:
            rospy.loginfo("Place found in map!")
            return True
        rospy.loginfo("Place not found in map :c")
        return False
      
    def execute_cb(self, goal):
        # Validate target location
        rospy.loginfo("Looking for the goal in the map...")
        goal_given = goal.target_location
        isValid = validateGoal(goal_given)       
        
        # Valid if the given location is in the known locations.
        if isValid == True:
            # Start executing the action
            self._as.set_active(False)
            send_goal(self.goals[goal_given])
        else:
            #Rejected goal
            self._result = False
            rospy.loginfo('%s: Aborted. Location not found' % self._action_name)
            self._as.set_aborted(self._result)

    def send_goal(self, goal_pose_given):
        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")
        
        goal = MoveBaseGoal()
        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # set up goal pose and orientation
        goal.target_pose.pose.position =  Point(goal_pose_given[0],goal_pose_given[1],0)
        goal.target_pose.pose.orientation.x = goal_pose_given[2]
        goal.target_pose.pose.orientation.y = goal_pose_given[3]
        goal.target_pose.pose.orientation.z = goal_pose_given[4]
        goal.target_pose.pose.orientation.w = goal_pose_given[5]

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(120))

        #move_base_result = move_base_client.get_result() 
        # while (move_base_result.result != 3 or move_base_result != 1):
        #     if move_base_result.result == 1:
        #         self._as.publish_feedback(self._feedback)
        #     else:
        #             self._result = False
        #             self._as.publish_feedback(None)
        #             self._as.set_succeeded(self._result)
       
        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            self._result = True
            self._as.set_succeeded(self._result)
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            self._result = False
            self._as.set_aborted(self._result)
            return False


if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()
