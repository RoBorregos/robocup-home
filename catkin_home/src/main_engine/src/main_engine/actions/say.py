from .Action import Action

class say(Action):
    def __init__(self, id, action_client_binded, args):
        Action.__init__(self,id,action_client_binded,args)
        print("Initializing pick_up Action instance")

    def run(self):
        # Call say.py service
        print("Running say action....")
        print(self.args)