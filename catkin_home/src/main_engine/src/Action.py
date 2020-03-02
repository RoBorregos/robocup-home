import rospy
import actionlib
#from object_detection.msg import RecognizeObjectAction, RecognizeObjectGoal
# TODO: Import all the messages related to actions I guess ?
class Action(object):
    def __init__(self,id,priority,action_server_responsible, args):
        self.id = id
        self.priority = priority
        self.args = []
        self.feedback = ["INIT", "IN PROCESS", "DONE"]
        self.action_server_client = None##actionlib.SimpleActionClient('action_server_responsible', RecognizeObjectAction)
