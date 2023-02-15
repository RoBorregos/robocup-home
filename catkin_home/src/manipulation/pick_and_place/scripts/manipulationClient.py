#! /usr/bin/env python3

import rospy
import actionlib
from object_manipulation.msg import manipulationServAction, manipulationServGoal
from geometry_msgs.msg import PoseStamped
from enum import Enum

OBJECTS_NAME= {
    1 : 'VEGETABLES',
    2: 'TEA',
    3 : 'COKE',
    4 : 'JUICE',
}
OBJECTS_ID= {
    'VEGETABLES' : 1,
    'TEA' : 2,
    'COKE' : 3,
    'JUICE' : 4,
}
class ManipulationGoals(Enum):
    VEGETABLES = 1
    TEA = 2
    COKE = 3
    JUICE = 4

def generateObjectString():
    result = ""
    for object_id in OBJECTS_NAME:
        object_ = OBJECTS_NAME[object_id]
        result += str(object_id) + ") " + object_ + "\n"
    result += "0) Exit"
    return result 

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

class ManipulationClient(object):
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient('manipulationServer', manipulationServAction)
        self.client.wait_for_server()

        while True:
            x = handleIntInput(
                msg_ = generateObjectString(),
                range=(0, 4)
            )
            if x == 0:
                break
            self.manipulation_goal(ManipulationGoals(x))

    def manipulation_goal(self, target = ManipulationGoals.COKE):
        class ManipulationGoalScope:
            object_ = target
            result = False
            
            result_received = False
        
        def manipulation_goal_feedback(feedback_msg):
            pass
        
        def get_result_callback(state, result):
            ManipulationGoalScope.result = result.result

            ManipulationGoalScope.result_received = True
            rospy.loginfo("Manipulation Goal Finished")

        rospy.loginfo("Sending Manipulation Goal")
        self.client.send_goal(
                    manipulationServGoal(object_id = ManipulationGoalScope.object_.value),
                    feedback_cb=manipulation_goal_feedback,
                    done_cb=get_result_callback)
        
        while not ManipulationGoalScope.result_received:
            pass
        
        return ManipulationGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('ManipulationGoalClient', anonymous=True)
        rospy.loginfo("ManipulationGoalClient initialized.")
        ManipulationClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
