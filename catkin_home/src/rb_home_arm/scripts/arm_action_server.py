#! /usr/bin/env python

import rospy

import actionlib

import rb_home_arm.msg

class ArmAction(object):
    # create messages that are used to publish feedback/result
    _pick_up_feedback = rb_home_arm.msg.Pick_upFeedback()
    _pick_up_result = rb_home_arm.msg.Pick_upResult()

    def __init__(self, name, action):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Started arm action server "+ self._action_name)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        rospy.loginfo("Executing callback")
        
if __name__ == '__main__':
    rospy.init_node('arm_action_server')
    pick_up_server = ArmAction("pick_up", rb_home_arm.msg.Pick_upAction)
    put_down_server = ArmAction("put_down", rb_home_arm.msg.Put_downAction)
    rospy.spin()