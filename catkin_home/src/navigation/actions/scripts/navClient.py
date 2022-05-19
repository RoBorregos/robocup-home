#! /usr/bin/env python3

import rospy
import actionlib
from actions.msg import navServAction, navServGoal
from geometry_msgs.msg import PoseStamped
from enum import Enum

def handleIntInput(msg_ = "", range=(0, 10)):
    x = -1
    while x < range[0] or x > range[1]:
        print(msg_)
        while True:
            x = input()

            if x and x.isnumeric():
                break
        x = int(x)
    return x

class MoveGoals(Enum):
    KITCHEN = 1
    COUCH = 2
    BATHROOM = 3
    CLOSET = 4

class NavClient(object):
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient('navServer', navServAction)
        self.client.wait_for_server()

        while True:
            x = handleIntInput(
                msg_ = "1. Kitchen  \n 2. Couch  \n 3. Bathroom  \n 4. Closet  \n 0. Exit",
                range=(0, 4)
            )
            if x == 0:
                break
            self.nav_goal(MoveGoals(x))

    def nav_goal(self, target = MoveGoals.KITCHEN):
        class NavGoalScope:
            target_location = target.name
            result = False
            pose = PoseStamped()
            
            result_received = False
        
        def nav_goal_feedback(feedback_msg):
            NavGoalScope.pose = feedback_msg.pose
        
        def get_result_callback(state, result):
            NavGoalScope.result = result.result

            NavGoalScope.result_received = True
            rospy.loginfo("Nav Goal Finished")

        rospy.loginfo("Sending Nav Goal")
        self.client.send_goal(
                    navServGoal(target_location = NavGoalScope.target_location),
                    feedback_cb=nav_goal_feedback,
                    done_cb=get_result_callback)
        
        while not NavGoalScope.result_received:
            pass
        
        return NavGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('NavGoalClient', anonymous=True)
        rospy.loginfo("NavGoalClient initialized.")
        NavClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
