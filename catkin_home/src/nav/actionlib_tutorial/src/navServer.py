#! /usr/bin/env python

import rospy
import actionlib
import json

import actionlib_tutorials.msg
import actionlib_tutorial.msg
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction

class navigationServer(object):
    # create messages that are used to publish feedback/result
   # _feedback = actionlib_tutorials.msg.FibonacciFeedback()
   # _result = actionlib_tutorials.msg.FibonacciResult()
    _feedback = actionlib_tutorial.msg.navServFeedback()
    _result = actionlib_tutorial.msg.navServResult()
    x=0
    y=0
    z=0
    qx=0
    qy=0
    qz=0
    qw=0


    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorial.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def searchGoal(self, goal):
       
        i=""
        with open('src/actionlib_tutorial/src/locations.json') as x:
            goals = json.load(x)

        for pos in goals:
            if(pos == goal.order):
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
        # helper variables
        r = rospy.Rate(1)
        success = True
        g = rospy.Publisher('goal', MoveBaseGoal, queue_size=10)
        
        # start executing the action
        self._feedback.pose = PoseStamped()
        rospy.loginfo("Looking for the goal")
        isValid = self.searchGoal(goal)

        if self._as.is_preempt_requested():

            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
            
        r.sleep()
        
        if isValid == True:
            rospy.loginfo("Goal found!")
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            rospy.loginfo("Waiting for server response")
            #client.wait_for_server()
            #rospy.loginfo("Server found")

            target = MoveBaseGoal()
            target.target_pose.header.frame_id = "goal"
            target.target_pose.header.stamp = rospy.Time.now()

            target.target_pose.pose.position.x = self.x
            target.target_pose.pose.position.y = self.y
            target.target_pose.pose.position.z = self.z

            target.target_pose.pose.orientation.x = self.qx
            target.target_pose.pose.orientation.y = self.qy
            target.target_pose.pose.orientation.z = self.qz
            target.target_pose.pose.orientation.w = self.qw

            g.publish(target)

            rospy.loginfo("Sending Goal")
            #client.send_goal(target)
            #rospy.loginfo("Goal sent!")
            #client.wait_for_result()
            #rospy.loginfo("Waiting for result!")
            #sate = actionlib.SimpleActionClient.get_state()
            #rospy.loginfo("State: %s", str(sate))
            #result =  client.get_result

            self._result = result
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._result = result
            rospy.loginfo('%s: Aborted' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()