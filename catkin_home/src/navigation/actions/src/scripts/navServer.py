#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import json

import navigation.msg
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction

class navigationServer(object):
    # Create messages that are used to publish feedback/result
    _feedback = navigation.msg.navServFeedback()
    _result = navigation.msg.navServResult()

    # Initialize posestamped variables
    x=0
    y=0
    z=0
    qx=0
    qy=0
    qz=0
    qw=0


    def __init__(self, name):
        self._action_name = name
        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, navigation.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def searchGoal(self, goal):

       # Load data
        i=""
        with open('src/navigation/actions/src/data/mock_locations.json') as x:
            goals = json.load(x)

        for pos in goals:
            # goal.target_location -> the location sent by the main_engine (specify the type on nav_action/navServ.action)
            if(pos == goal.target_location): 
                i  = goals[pos]["location"]

       
        if(i == ""): return False

        self.x = i["x"]
        self.y = i["y"]
        self.z = i["z"]
        self.qx = i["qx"]
        self.qy = i["qy"]
        self.qz = i["qz"]
        self.qw = i["qw"]

        return True
      
    def execute_cb(self, goal):
        # Helper variables
        r = rospy.Rate(1)
        success = True
        g = rospy.Publisher('goal', MoveBaseGoal, queue_size=10)
        
        # Start executing the action
        # self._feedback.status = True
        rospy.loginfo("Looking for the goal")
        isValid = False
        print(type(goal.target_location))
        if(type(goal.target_location) == str):
            isValid = self.searchGoal(goal)
        #else:
            #go to object action
        if self._as.is_preempt_requested():

            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        
        # Valid if the given location is in the known locations.
        if isValid == True:
            rospy.loginfo("Goal found!")

            # Look if the move_base node is up
            move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            rospy.loginfo("Waiting for server response")

            #move_base_client.wait_for_server()
            #rospy.loginfo("Server found")

            # Creating the goal message (PoseStamped)
            target = MoveBaseGoal()
            target.target_pose.header.frame_id = "goal"
            target.target_pose.header.stamp = rospy.Time.now()

            # xyz positions
            target.target_pose.pose.position.x = self.x
            target.target_pose.pose.position.y = self.y
            target.target_pose.pose.position.z = self.z
            
            # Quaternion positions
            target.target_pose.pose.orientation.x = self.qx
            target.target_pose.pose.orientation.y = self.qy
            target.target_pose.pose.orientation.z = self.qz
            target.target_pose.pose.orientation.w = self.qw

            while True:
                g.publish(target)
                rospy.sleep(1)

            rospy.loginfo("Sending Goal to move base node")
            #move_base_client.send_goal(target)
            #rospy.loginfo("Goal sent!")
            #client.wait_for_result()
            #move_base_result = move_base_client.get_result() 
            while (move_base_result.result != 3 or move_base_result != 1):
                if move_base_result.result == 1:
                    self._as.publish_feedback(self._feedback)
                else:
                     self._result = False
                     self._as.publish_feedback(None)
                     self._as.set_succeeded(self._result)    

            self._result = success
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._result = success
            rospy.loginfo('%s: Aborted. Location not found' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()
