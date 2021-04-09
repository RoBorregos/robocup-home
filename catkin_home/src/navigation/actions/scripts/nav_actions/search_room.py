#! /usr/bin/env python
from __future__ import print_function

import csv
import json
import actionlib
import rospy

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_actions.move_base import MoveBase

class searchRoom(object):
    _path = []

    def getGoalPoints(self, location):
        ## TODO 
        ## Implement get_room_path
        ## self._path = get_room_path.getPoints(location)
        ## return get_room_path.generatePath(self._path)
        with open('goals.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                goal = [float(i) for i in row]
                self._path.append(goal)
            rospy.loginfo(self._path)
        return self._path

if __name__ == "__main__":
    searchRoom = searchRoom()