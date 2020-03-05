import rospy
import actionlib
import time
#from object_detection.msg import RecognizeObjectAction, RecognizeObjectGoal
# TODO: Import all the messages related to actions I guess ?
'''
All Goals, and Actions must be imported here from he corresponding packages
'''


class Action(object):
    def __init__(self, id, priority, action_client_binded, category, args):
        self.id = id
        self.priority = int(priority)
        self.args = args
        self.category = int(category)
        #Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
        self.feedback = ["PENDING", "ACTIVE", "RECALLED", "REJECTED", "PREEMPTED", "ABORTED", "SUCCEEDED", "LOST"]
        self.feedback_step = 0
        self.action_client_name = action_client_binded 
        self.action_client  = None
        self.time_requested = int(time.time())
        self.specific_function ={
            "bring_something":self.bring_something,
            "go_to":self.go_to,
            "default": self.specific_function_not_found,
        }
        self.specific_function.setdefault("default")


    def specific_function_not_found(self):
        print("Not found the corresponding action")

    def print_self(self, endline):
        if(endline == False):
            print(self.get_description(),)
            return
        print(self.get_description())

    def get_description(self):
        return self.id + "-" + str(self.priority) + "-" + str(self.category) + "-"+str(self.time_requested)+" : " + self.feedback[self.feedback_step] + " "

    def stop(self):  
        print("Stop Action")

    def run(self):
        print("Run Action")
        self.specific_function[self.id]()


    def run_action_client(self, ROS_action):
        print("Booting action client")
        #self.action_client = actionlib.SimpleActionClient(self.action_client_name, ROS_action)
        #self.action_client.wait_for_server()
        return True
        
        '''Specific Goals and Results'''
    def bring_something(self):
        print("Setting goal,filling it and contacting action server of bring_something")
        #action.run_action_client(BringSomethingAction)
        #goal = BringSomethingGoal()
        #goal.target_location = "kitchen"
        #goal.target_object = "juguito"
        #action.send_goal(goal)
    
    def go_to(self):
        print("Setting goal,filling it and contacting action server of bring_something")
        #action.run_action_client(GoToAction)
        #goal = GoToGoal()
        #goal.target_location = "kitchen"
        #action.send_goal(goal)