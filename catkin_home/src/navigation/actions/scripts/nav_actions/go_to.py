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

class goToAction(object):
    # goals = {}

    def __init__(self, goal):
        self.goal = goal
        # self.setLocations()
    
    # def setLocations(self):
    #     # declare the coordinates of interest
    #     a = ['entrance', 'bedroom', 'kitchen', 'restroom', 'dinning_room']
    #     place = 0
    #     self.goals = {}
    #     with open('src/actions/data/goals.csv', 'r') as file:
    #         reader = csv.reader(file)
    #         for row in reader:
    #             goal = [float(i) for i in row]
    #             self.goals[a[place]] = goal
    #             place=(place+1)%len(a)
    #         rospy.loginfo(self.goals)
    
    # def locationExists(self,location):
    #     if type(location) != str:
    #         return False
    #     rospy.loginfo("Place received: %s", location)
    #     if location in self.goals:
    #         rospy.loginfo("Place found in map!")
    #         return True
    #     rospy.loginfo("Place not found in map :c")
    #     return False
    
    # def getLocation(self,location):
    #     return self.goals[location]
    
    def getGoals(self):
        return self.goals

if __name__ == '__main__':
    action = goToAction()