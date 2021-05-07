import rospy
import actionlib
import time
import sys
import rb_home_arm.msg

'''
All Goals, and Actions must be imported here from he corresponding packages
'''


class Action(object):
    def __init__(self, id, action_client_binded, args):
        self.id = id
        self.args = args
        # Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
        self.feedback = ["PENDING", "ACTIVE", "RECALLED",
                         "SUCCEEDED", "PREEMPTED", "ABORTED", "REJECTED", "LOST"]
        self.action_client_name = action_client_binded
        self.action_client = None
        self.time_requested = int(time.time())

    def specific_function_not_found(self):
        print("Not found the corresponding action")

    def print_self(self, endline=False):
        if(endline == False):
            print(self.get_description(),)
            return
        print(self.get_description())

    def get_description(self):
        return self.id + "-"+ str(self.time_requested) +  "-" + self.args +  " : " + self.feedback[self.get_state()]

    def stop(self):
        print("Stop Action")

    def run_action_client(self, ROS_action):
        print("Booting action client, waiting...")
        self.action_client = actionlib.SimpleActionClient(self.action_client_name, ROS_action)
        self.action_client.wait_for_server()
        return True

    def get_feedback(self):
        #TODO: Validation of get_state
        return self.feedback[self.get_state()]

    def get_state(self):
        if(self.action_client):
            state = 7
            ATTEMPTS = 2
            #Contacts Action Server to get state of action
            for i in range(0, ATTEMPTS):
                try:
                    state =  self.action_client.get_state() #Returns an int apparently (?)
                    break
                except:
                    print("Failed attempt", str(i), "Error:", str(sys.exc_info()[0]))
            print(state)
        else:
            state=0
        return state

    def run_action_client(self, ROS_action):
        print("Booting action client, waiting...")
        self.action_client = actionlib.SimpleActionClient(self.action_client_name, ROS_action)
        self.action_client.wait_for_server()
        return True
        
