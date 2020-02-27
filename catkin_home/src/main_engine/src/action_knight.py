class ActionKnight(object):
    def __init__(self, possibleActions,cmd_id,cmd_priority):
        self.cmd_id = cmd_id
        self.cmd_priority = cmd_priority
        self.action = None
        self.place = None
        self.object = None
        self.subject = None
        self.stage = None
        self.stages = ["INIT", "IN PROCESS", "DONE"]
        #Que se cargen los sinonimos de un archivo
        self.associatedNode = None
        self.possibleActions = possibleActions

    def fetchPossibleActions(self):
        print("Getting possible actions from csv....")
