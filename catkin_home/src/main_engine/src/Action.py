import rospy
import actionlib
import time
import sys
import actions.msg
import rb_home_arm.msg
# TODO: Import all the messages related to actions I guess ?
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
        self.specific_function = {
            "bring_something": self.bring_something,
            "go_to": self.go_to,
            "pick_up":self.pick_up,
            "put_down":self.put_down,
            "default": self.specific_function_not_found,
        }
        self.specific_function.setdefault("default")

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

    def run(self):
        print("Run Action")
        print(self.id)
        self.specific_function[self.id]()

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
                    print "Failed attempt " +str(i) +"Error: " + str(sys.exc_info()[0])
            print(state)
        else:
            state=0
        return state


        '''Specific Goals and Results'''
    def bring_something(self):
        print("Setting goal,filling it and contacting action server of bring_something")
        # self.run_action_client(BringSomethingAction)
        #goal = BringSomethingGoal()
        #goal.target_location = "kitchen"
        #goal.target_object = "juguito"
        # action.send_goal(goal)

    def go_to(self):
        print("Setting goal,filling it and contacting action server of bring_something")
        GoToAction = actions.msg.navServAction
        self.run_action_client(GoToAction)
        goal = actions.msg.navServGoal()
        goal.target_location = "kitchen"
        self.action_client.send_goal(goal)

    def pick_up(self):
        print("Setting goal,filling it and contacting action server of pick_up")
        ArmAction = rb_home_arm.msg.ArmAction
        self.run_action_client(ArmAction)
        print("Done initial contact with server")
        goal = rb_home_arm.msg.ArmGoal()
        goal.object = self.args
        goal.type_of_movement = "pick_up"
        self.action_client.send_goal(goal)
    
    def put_down(self):
        print("Setting goal,filling it and contacting action server of put_down")
        ArmAction = rb_home_arm.msg.ArmAction
        self.run_action_client(ArmAction)
        print("Done initial contact with server")
        goal = rb_home_arm.msg.ArmGoal()
        goal.object = self.args
        goal.type_of_movement = "put_down"
        self.action_client.send_goal(goal)