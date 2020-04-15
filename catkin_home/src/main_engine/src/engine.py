#!/usr/bin/env python

# DESCRIPTION
from intercom.msg import action_selector_cmd
import time
from threading import Thread
import rospy
"""
	This scrips simulates the reception of a cmd and asigns it
	to a queue, called -actionQueue-.
"""

# TOPICS PUBLISHED
"""
    NA
"""

# TOPICS READ
"""
    engine_commands -> Byte
"""

# MSG USED
"""
    action_selector_msg
        Header header
        uint8 cmd_id
        uint8 cmd_priority
        uint8 critic_shutdown
"""


isActionQueueChanged = True
actionQueue = []


def actionQueueGUI():
    global   isActionQueueChanged
    def printQueueStatus():
        print "  === actionQueue ==="
        print "\t+--^--+"
        for x in xrange(len(actionQueue)):
            print "\t| ", actionQueue[x], " |"
        print "\t+-----+"
        time.sleep(0.5)
    
    # ROS loop
    while (not rospy.is_shutdown()):
        # if Action Queue Changed, print it
        if (isActionQueueChanged):
            printQueueStatus()
            isActionQueueChanged = False


def process_id(msg):
    global isActionQueueChanged
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if (msg.critic_shutdown == 1):
        rospy.logfatal("Critic shutdown signal from Action Selector received")

    actionQueue.append(msg.cmd_id)
    isActionQueueChanged = True


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('engine', anonymous=False)

    rospy.Subscriber("engine_commands", action_selector_cmd, process_id)

    # utility function for viewing actionQueueGUI
    actionQueueGUI()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
