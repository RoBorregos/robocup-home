import rb_home_arm.msg
from .Action import Action

class put_down(Action):
    def __init__(self, id, action_client_binded, args):
        Action.__init__(self,id,action_client_binded,args)
        print("Initializing put_down action instance")

    def run(self):
        print("Setting goal,filling it and contacting action server of put_down")
        ArmAction = rb_home_arm.msg.ArmAction
        self.run_action_client(ArmAction)
        print("Done initial contact with server")
        goal = rb_home_arm.msg.ArmGoal()
        goal.object = self.args
        goal.type_of_movement = "put_down"
        self.action_client.send_goal(goal)