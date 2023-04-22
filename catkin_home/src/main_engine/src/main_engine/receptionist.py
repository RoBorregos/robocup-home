#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from action_selectors.msg import RawInput

import os
import openai


promts = {
    "start": "Hello, I'm homie, let's start receptionist task",
    "wait": "I am waiting for someone",
    "self_intro": "Hello, I'm homie, your personal assistant. How can I help you?",
    "ask_name": "What's your name?",
    "ask_drink": "What is your favorite drink you like to drink?",
}

calls = {
    "get_name": "get me the name of the person on the next message:"

}





INITIALIZATION_STATE = "init"
START_STATE = "start"
WAIT_STATE = "waiting for someone"
IDENTIFY_STATE = "identifying the person"
GOING_STATE = "going somewhere"
ASSIGN_STATE = "assing a seat to someone"
RETURN_STATE = "returning to the reception"
END_STATE = "done"

class receptionist(object):
    def __init__(self):
        openai.api_key = os.getenv("OPENAI_API_KEY")


        self.currentState =  INITIALIZATION_STATE

        # GPT API
        self.GPT_API = openai.Completion.create(
            model="text-davinci-003",
            prompt="",
            temperature=0.7,
            max_tokens=1024,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0
            )
        rospy.logdebug("GPT API initialized")
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.speech_enable.publish(Bool(False))

        #Nav subscribers 
        #self.cancel_move = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        #self.initial_pose = None
        #self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)
        self.is_moving = False

        rospy.loginfo("Receptionist initialized")

        # Conversation dependencies
        
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=20)
        self.input_suscriber = rospy.Subscriber("RawInput", RawInput, self.input_callback)


        self.currentState = START_STATE
        # wait for the robot to be ready
        time.sleep(1)
        # start the main loop
        self.run()


    
    def run(self):

        while not rospy.is_shutdown():

            if self.currentState == START_STATE:
                self.speech_enable.publish(Bool(False))

                rospy.loginfo("Receptionist is ready to start")
                self.say_publisher.publish(promts["start"])
                time.sleep(3)

                rospy.loginfo("Receptionist is waiting for user to start")
                time.sleep(3)
                self.say_publisher.publish(promts["self_intro"])

                self.currentState = WAIT_STATE

                
            elif self.currentState == WAIT_STATE:
                if self.is_someone():
                    self.currentState = IDENTIFY_STATE
                else:
                    time.sleep(3)
                    self.say_publisher.publish(promts["wait"])
                
            elif self.currentState == IDENTIFY_STATE:
                name = self.get_name()                



    def get_name(self):
        # get the name from the user
        rospy.loginfo("Receptionist is getting name")
        self.say_publisher.publish(promts["ask_name"])
        response = self.parser(self)
        rospy.logwarn(response)

        time.sleep(3)
        return "Ahmed"
    


    def is_someone(self):
        # check for a new detection in vision topic
        rospy.loginfo("checking if someone is there")
        time.sleep(5)
        return False



    def parser(self):
        rospy.loginfo("Receptionist is parsing")
        try:
            intent = self.callGPT(self, self.inputText)

        except:
            self.say("I'm sorry, Could you rephrase?")
            self.debug("Failed response")
        
        return intent

            

    def input_callback(self, msg):
            inputText = msg.inputText
            self.debug("I heard: " + inputText)
            self.parser(self)
            
    def callGPT(self, prompt):
        response = openai.Completion.create(
            engine=self.GPT_API.engine,
            prompt=prompt,
            max_tokens=self.GPT_API.max_tokens,
            n=1,
            temperature=self.GPT_API.temperature,
            frequency_penalty=self.GPT_API.frequency_penalty,
            presence_penalty=self.GPT_API.presence_penalty
        )
        return response.choices[0].text.strip()


                    


        

        



def main():
    rospy.init_node('receptionist', anonymous=True)
    rospy.loginfo("Receptionist task created.")
    receptionist()
    rospy.spin()


if __name__ == '__main__':
    main()