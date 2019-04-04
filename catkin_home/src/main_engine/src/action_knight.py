class ActionKnight(object):
    def __init__(self):
        self.cmd_id=cmd_id
        self.cmd_priority=cmd_priority
        self.action = None
        self.place = None
        self.object = None
        self.subject = None
        self.stage = None
        self.stages = ["INIT","IN PROCCESS", "DONE"] 
        #Que se cargen los sinonimos de un archivo
        self.preProcess()
        
	def preProcess(self):
        self.stage = self.stages[0]
        self.fetchPossibleActions()
                
    def fetchPossibleActions():
        print("Getting possible actions from csv....")