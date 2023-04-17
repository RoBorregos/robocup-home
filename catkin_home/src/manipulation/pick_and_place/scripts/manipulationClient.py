#! /usr/bin/env python3

import rospy
import actionlib
from pick_and_place.msg import manipulationServAction, manipulationServGoal
from geometry_msgs.msg import PoseStamped
from enum import Enum
import time
import signal

OBJECTS_NAME= {
    1 : 'Coca-Cola',
    2 : 'Coffee',
    3 : 'Nesquik',
    4 : 'Zucaritas',
    5 : 'Harpic'
}
OBJECTS_ID= {
    'Coca-Cola' : 1,
    'Coffee' : 2,
    'Nesquik' : 3,
    'Zucaritas' : 4,
    'Harpic' : 5
}
class ManipulationGoals(Enum):
    COKE = 1
    COFFEE = 2
    NESQUIK = 3
    ZUCARITAS = 4
    HARPIC = 5
    BIGGEST = 6

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
        rospy.loginfo("Connecting to Manipulation Server")
        self.client = actionlib.SimpleActionClient('manipulationServer', manipulationServAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to Manipulation Server")

        result = False
        x = OBJECTS_ID['Coca-Cola']
        while not result and not rospy.is_shutdown():
            if x == 0:
                break
            result = self.manipulation_goal(ManipulationGoals(x))
            ## Wait for user input
            in_ = handleIntInput("(1) Zucaritas, (2) Coca-Cola, (3) Harpic", range=(0, 3))
            if in_ == 0:
                break
            if in_ == 1:
                x = OBJECTS_ID['Zucaritas']
            elif in_ == 2:
                x = OBJECTS_ID['Coca-Cola']
            elif in_ == 3:
                x = OBJECTS_ID['Harpic']
            

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
        
        while not ManipulationGoalScope.result_received and not rospy.is_shutdown():
            pass
        
        return ManipulationGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('ManipulationGoalClient', anonymous=True)
        rospy.loginfo("ManipulationGoalClient initialized.")
        ManipulationClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
