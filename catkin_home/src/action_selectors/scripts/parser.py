#!/usr/bin/env python
import rospy
from action_selectors.msg import RawInput
from intercom.msg import action_selector_cmd

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.inputText)

    #open list of actions
    #call parser
    #send message to engine 
    pub = rospy.Publisher('action_selector_cmds', action_selector_cmd, queue_size=10)
    action_code = action_selector_cmd()
    action_code.cmd_id = 1
    action_code.cmd_priority = 1
    action_code.critic_shutdown = 0
    action_code.args = ["kitchen"]
    rospy.loginfo(action_code)
    pub.publish(action_code)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('parser', anonymous=True)

    rospy.Subscriber("RawInput", RawInput, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()