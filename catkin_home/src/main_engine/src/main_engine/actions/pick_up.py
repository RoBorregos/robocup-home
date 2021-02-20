import rb_home_arm.msg
from .Action import Action

class pick_up(Action):
    def __init__(self, id, action_client_binded, args):
        Action.__init__(self,id,action_client_binded,args)
        print("Initializing pick_up Action instance")

    def run(self):
        print("Setting goal,filling it and contacting action server of pick_up")
        ArmAction = rb_home_arm.msg.ArmAction
        self.run_action_client(ArmAction)
        print("Done initial contact with server")
        goal = rb_home_arm.msg.ArmGoal()
        goal.object = self.args
        goal.type_of_movement = "pick_up"
        self.action_client.send_goal(goal)