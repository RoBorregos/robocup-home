#! /usr/bin/env python
from __future__ import print_function

import csv
import json
import math
import tf

import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_actions.go_to import goToAction
from nav_actions.move_base import MoveBase
from nav_actions.search_room import searchRoom

import actions.msg

""" 
Valid states:
    PENDING
    ACTIVE
    SUCCEEDED
    ABORTED
Valid Actions:
    gt-Place
    so-Object
    ao-toDefine
"""

class navigationServer(object):
    # Create messages that are used to publish feedback/result
    _feedback = actions.msg.navServFeedback()
    _result = actions.msg.navServResult()

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)
        self.move_base_status = 0
        # Create a topic listener of move_base node status
        moveBaseStatusTopic = rospy.Subscriber("move_base/status", GoalStatusArray, self.setServerFeedback)
        self.pub_amcl = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.context_map = None
        with open("./src/navigation/map_contextualizer/contextmaps/DemoMap.json", "r") as read_file:
            self.context_map = json.load(read_file)
        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        # Action Type
        action = goal.target_location[0:2]
 
        if action == "gt" or action == "so":
            # Validate target location
            rospy.loginfo("Room received!")
            rospy.loginfo("Looking for the goal in the map...")
            room = goal.target_location[3:]
            if room not in self.context_map["rooms"]:
                rospy.loginfo("The room does not exist.")
                self._result = False
                self._as.set_aborted()
                return

            # Check if robot is inside room with ray casting
            if self.robotInsideRoom(room) and action == "gt":
                rospy.loginfo("Robot is already inside the room.")
                self._result = true
                self._as.set_succeeded(self._result)
                return

            rospy.loginfo("Room received!")
            # TODO: send position to nearest entrance
            entrance = self.context_map["rooms"][room]["entrance"][0]
            actionResult = self.send_goal(entrance)
            if not actionResult:
                rospy.loginfo("The robot failed to reach the destination")
                self._result.result = False
                self._as.set_aborted()
                return
                
            if action == "gt":
                rospy.loginfo("You have reached the destination")
                self._result.result = True
                self._as.set_succeeded(self._result)
                return
            else: # action == "so":
                path = self.context_map["rooms"][room]["path"]
                # i = 0
                # j = i + 1
                for goalPoint in path:
                    # direction = (path[j][0]-path[i][0], path[j][1]-path[i][1])
                    result = self.send_goal(goalPoint)
                    if (not result):
                        rospy.loginfo("The robot failed to reach the destination")
                        self._result.result = False
                        self._as.set_aborted()
                        return
                    rospy.loginfo('Arrived to Goal Point')
                rospy.loginfo("Room searched.")
                self._result.result = True
                self._as.set_succeeded(self._result)                
        elif action == "ao":
            rospy.loginfo("Approach Object Action (TODO)")
            self._result.result = True
            self._as.set_succeeded(self._result)
        elif action == "cl":
            currentAction = moveBase()
            currentAction.cancelGoal() 
        else:
            rospy.loginfo("Invalid Action")
            self._result = False
            self._as.set_aborted()

    def robotInsideRoom(self, room):
        # Get robot position
        listener = tf.TransformListener()
        trans, rot = [None, None]
        listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('map', 'base_link', now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Cannot get location of robot!")
            self._result = false
            self._as.set_aborted(self._result)
            return
            return 1

        testy = trans[1]
        testx = trans[0]
        area_points = self.context_map["rooms"][room]["area"]
        nvert = len(area_points)
        i, c = [0,0]
        j = nvert-1
        for _ in range(nvert):
            if ( ((area_points[i][1]>testy) != (area_points[j][1]>testy)) and (testx < (area_points[j][0]-area_points[i][0]) * (testy-area_points[i][1]) / (area_points[j][1]-area_points[i][1]) + area_points[i][0]) ):
                c = not c
            j = i
            i+=1
        return c
    
    # def executeGoToAction(self, location):
    #     # Start executing the action
    #     actionResult = self.send_goal(location)
    #     if actionResult:
    #         rospy.loginfo("You have reached the destination")
    #         self._result.result = True
    #         self._as.set_succeeded(self._result)
    #     else:
    #         rospy.loginfo("The robot failed to reach the destination")
    #         self._result.result = False
    #         self._as.set_aborted()
    
    # def executeSearchRoomAction(self, room):
    #     searchRoomAction = searchRoom()
    #     path = searchRoomAction.getGoalPoints(room)

    #     for goalPoint in path:
    #         result = self.send_goal(goalPoint)
    #         if (not result):
    #             rospy.loginfo("The robot failed to reach the destination")
    #             self._result.result = False
    #             self._as.set_aborted()
    #             return
    #         rospy.loginfo('Arrived to Goal Point')

    #     rospy.loginfo("You have reached the destination")
    #     self._result.result = True
    #     self._as.set_succeeded(self._result)
    #     ## loop over all goal points until end of room or object found
    #     ## for i in range(len(goalPoints)):
    #         ## self.send_goal(i)

    # Sets the server's feedback based on the move base feedback
    def setServerFeedback(self, data):
        #rospy.loginfo("list: %s", self.tf.getFrameStrings())

        #if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
         #   rospy.loginfo("true")
          #  timeStamp = self.tf.getLatestCommonTime("/base_link", "/map")
           # currentPos, quat = self.tf.lookupTransform("/base_link", "/map", timeStamp)
        if len(data.status_list):
            self.move_base_status = data.status_list[0].status
            # print(data.status_list[0].status)
        # self._feedback.status = self._goal.getMoveBaseStatus(data)

        # self._as.publish_feedback(self._feedback)
    
    def send_goal(self, goal_pose_given):
        rospy.loginfo(goal_pose_given)
        # self._goal = MoveBase()
        # self._goal.setGoal(goal_pose_given)
        # self.tf = tf.TransformListener(True, rospy.Duration(3.0))
        # rospy.loginfo("Sending goal location ...")
        # moveBaseState = self._goal.sendGoalToNavStack()
        # moveBaseStatusTopic.unregister()

        # return moveBaseState
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()
        point = Point()
        point.x = goal_pose_given[0]
        point.y = goal_pose_given[1]
        point.z = 0.0

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # TODO: Send correct Quat depending on the robot's direction
        goal.target_pose.pose = Pose(point, Quaternion(0.0, 0.0, -0.698491025714, 0.715618814032))

        client.send_goal(goal)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        #     return None
        # else:
        while not self.move_base_status == 1:
            print("Waiting for move_base to start")
            rospy.sleep(2)
        rate = rospy.Rate(10.0)
        trans, rot = None, None
        distance = 1
        self.move_base_status = 1
        listener = tf.TransformListener()
        while self.move_base_status == 1:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link',rospy.Time(0))
                prev_trans = trans
                prev_rot = rot
                distance = math.sqrt(pow((point.x - trans[0]),2)+ pow((point.y - trans[1]),2))
                if distance < 0.3:
                    print("Close enough")
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        if distance > 0.3:
            print("Error occured in movebase")
            return False
        else:
            client.cancel_goal()
            print("Reached position.")
            return True


if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()
