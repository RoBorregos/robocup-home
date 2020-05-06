#! /usr/bin/env python

import rospy

import actionlib

import rb_home_arm.msg

class ArmAction(object):
    # create messages that are used to publish feedback/result
    _feedback = rb_home_arm.msg.ArmFeedback()
    _result = rb_home_arm.msg.ArmResult()
    MAX_POS = 20

    def __init__(self, name, action):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Started arm action server "+ self._action_name)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        for i in range(0, self.MAX_POS):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
            self._feedback.position_of_arm = i
            rospy.loginfo("Current status of action: "+str(i))
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.done = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        rospy.loginfo("Executing callback")
        
if __name__ == '__main__':
    rospy.init_node('arm_action_server')
    print(rospy.get_name())
    Arm_server = ArmAction(rospy.get_name(), rb_home_arm.msg.ArmAction)
    #put_down_server = ArmAction("put_down", rb_home_arm.msg.Put_downAction)
    rospy.spin()