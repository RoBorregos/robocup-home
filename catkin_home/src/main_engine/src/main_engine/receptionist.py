#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
import os
import openai


info ={
    "type": "here it can be person/object/place",
    "identifier": "identifier",
    "place": "location if known, if not, blank",
    "pose": "Pose of the person/object/place",
    "status": "Null/Done/Doing",
}

INITIALIZATION_STATE = "init"
START_STATE = "start"
WAIT_STATE = "waiting for someone"
GOING_STATE = "going somewhere"
ASSIGN_STATE = "assing a sit to someone"
RETURN_STATE = "returning to the reception"
END_STATE = "done"

class receptionist(object):
    def __init__(self):

        self.currentState =  INITIALIZATION_STATE

        # GPT API
        self.GPT3_API = openai.Completion.create(
                        model="text-davinci-003",
                        prompt="",
                        temperature=0.7,
                        max_tokens=256,
                        top_p=1,
                        frequency_penalty=0,
                        presence_penalty=0
                        )
        
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)

        #Nav subscribers 
        #self.cancel_move = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        #self.initial_pose = None
        #self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)
        self.is_moving = False

        rospy.loginfo("Receptionist initialized")
        self.currentState = START_STATE
        self.run()
    
    def run(self):

        while not rospy.is_shutdown():
            if self.currentState == START_STATE:
                self.speech_enable.publish(Bool(True))



                rospy.loginfo("Receptionist is ready to start")
                self.currentState = WAIT_STATE

                
            elif self.currentState == WAIT_STATE:
                self.speech_enable.publish(Bool(True))


    def parser(self, data):
        rospy.loginfo("Receptionist is parsing")
        self,
            
                
                    


        

        

