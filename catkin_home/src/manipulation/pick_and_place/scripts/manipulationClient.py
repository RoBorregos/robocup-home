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
}
OBJECTS_ID= {
    'Coca-Cola' : 1,
    'Coffee' : 2,
    'Nesquik' : 3,
}
class ManipulationGoals(Enum):
    COKE = 1
    COFFEE = 2
    NESQUIK = 3

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
        while not result:
            x = OBJECTS_ID['Coca-Cola']
            if x == 0:
                break
            result = self.manipulation_goal(ManipulationGoals(x))
            time.sleep(5.0)

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
        
        stop_flag = False
        while not ManipulationGoalScope.result_received and not stop_flag:
            time.sleep(0.1)
            if signal.getsignal(signal.SIGINT):
                stop_flag = True
        
        return ManipulationGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('ManipulationGoalClient', anonymous=True)
        rospy.loginfo("ManipulationGoalClient initialized.")
        ManipulationClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)