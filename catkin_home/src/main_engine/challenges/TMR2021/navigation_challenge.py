#!/usr/bin/env python3

import roslaunch
import os
import sys
import rospy
from intercom.msg import action_selector_cmd

def main():
    # For this challenge just launch navigation and main_engine stack
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print(os.getcwd())
    # TODO: Launch file of navigation?
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["./src/main_engine/launch/main_engine.launch"])
    launch.start()

    # Publish todo with argv[1]

    if(len(sys.argv) > 1):
        print(sys.argv[1])
        action_request = action_selector_cmd()
        action_request.intent = "go_to"
        action_request.args = sys.argv[1]
        action_request.action_client_binded = "navigation"
        rospy.loginfo(action_request)
        rospy.init_node("navigation_challenge_node")
        action_requested_pub = rospy.Publisher('action_requested',
                          action_selector_cmd, queue_size=10, latch=True)
        action_requested_pub.publish(action_request)
        rospy.spin()
    else:
        raise Exception("No target destination provided for go_to")


if __name__=="__main__":
    main()