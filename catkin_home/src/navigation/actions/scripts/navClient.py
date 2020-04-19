#! /usr/bin/env python

from __future__ import print_function

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the navigation action, including the
# goal message and the result message.
import actions.msg

def navigationClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (NavigationAction) to the constructor.
    client = actionlib.SimpleActionClient('navServer', actions.msg.navServAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print("Nav server running...")

    # Creates a goal to send to the action server.
    goal = actions.msg.navServGoal(target_location = "kitchen")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # The Navigation status result

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('nav_client_py')
        
        result = navigationClient()
        print("Result:")
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)