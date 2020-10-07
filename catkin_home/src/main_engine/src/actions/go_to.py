from Action import Action

class go_to(Action):
    def __init__(self, id, action_client_binded, args):
        Action.__init__(self,id,action_client_binded,args)
        print("Initializing GoTo Action instance")

    def run(self):
        print("Setting goal,filling it and contacting action server of go_to")
        # TODO: Import action message and send it to the actionServer
        