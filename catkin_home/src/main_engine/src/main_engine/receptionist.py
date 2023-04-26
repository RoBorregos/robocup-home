#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String, Bool
from action_selectors.msg import RawInput
from humanAnalyzer.msg import face_array

import os
import openai


promts = {
    "start": "Let's start receptionist task",
    "wait": "I am waiting for someone",
    "self_intro": "Hello, I'm homie, your personal assistant.",
    "ask_name": "What's your name?",
    "ask_drink": "What is your favorite drink?",
    "repeat": "i think i didn't get that, can you repeat it?",
}

calls = {
    "get_name": "get me the name of the person on the next message: ",
    "get_drink": "get me the name of a  drink on the next message: ",
    "describe": "describe a person with the next attributes: ",
    "confirm" : "tell me True or False, the next message is a confirmation:",
    "reached": "tell me True or False, the next message is a confirmation that you reached a place:",

}





INITIALIZATION_STATE = "init"
START_STATE = "start"
WAIT_STATE = "waiting for someone"
IDENTIFY_STATE = "identifying the person"
GETTING_DRINK= "getting his favorite drink"
GOING_STATE = "going somewhere"
ASSIGN_STATE = "assing a seat to someone"
RETURN_STATE = "returning to the reception"
END_STATE = "done"



class person(object):
    def __init__(self, name):
        self.id = None
        self.name = name
        self.drink = None
        self.seat = None
        self.age = None
        self.gender = None
        self.race = None

        



    

class receptionist(object):
    def __init__(self):
        openai.api_key = os.getenv("OPENAI_API_KEY")


        self.currentState =  INITIALIZATION_STATE

        # GPT API
        
        self.GPT_model="text-davinci-003",
        self.GPT_temperature=0.7,
        self.GPT_top_p=1,
        self.GPT_frequency_penalty=0,
        self.GPT_presence_penalty=0
            
        rospy.logdebug("GPT API initialized")
        self.inputText = ""
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.saying = False

        self.say_listener = rospy.Subscriber("saying", Bool, self.say_callback)
        self.speech_enable.publish(Bool(False))

        #Nav subscribers 
        #self.cancel_move = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        #self.initial_pose = None
        #self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)
        self.is_moving = False

        #Face detection

        self.faces_subscriber = rospy.Subscriber("/faces", face_array, self.face_callback)
        rospy.loginfo("Receptionist initialized")

        # Conversation dependencies
        
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=20)
        self.input_suscriber = rospy.Subscriber("RawInput", RawInput, self.input_callback)


        self.currentState = START_STATE
        # wait for the robot to be ready
        time.sleep(1)
        # start the main loop
        # 
        # Array of persons
        self.faces = None
        self.persons = [] 

        self.run()
        rospy.spin()


    def say_callback(self, data):
        self.saying = data.data
        rospy.logwarn("saying: " + str(self.saying))

    def face_callback(self, data):
        self.faces = data.faces

    def run(self):

        while not rospy.is_shutdown():
            rospy.loginfo("-----------------------------------------RUN-----------------------------------------")

            if self.currentState == START_STATE:
                self.speech_enable.publish(Bool(False))

                rospy.loginfo("Receptionist is ready to start")
                self.say_publisher.publish(promts["start"])
                time.sleep(1)

                rospy.logwarn("here it shiuld the speech end") 
                i = 0
                self.currentState = WAIT_STATE

                
            elif self.currentState == WAIT_STATE:
                if self.is_someone():
                    self.say_publisher.publish(promts["self_intro"])
                    self.currentState = IDENTIFY_STATE
                else:
                    #if there is no one, wait for 3 seconds and check again
                    time.sleep(1)
                    if self.saying == False:
                        self.saying == True
                        self.say_publisher.publish(promts["wait"])
                    time.sleep(1)
                        

                
            elif self.currentState == IDENTIFY_STATE:
                name = self.get_name()
                if name != "":
                    self.currentState = GOING_STATE
                else:
                    self.currentState = IDENTIFY_STATE

            elif self.currentState == GOING_STATE:
                rospy.loginfo("Receptionist is going to the living room")
                self.say_publisher.publish("Come with me " + name)
                time.sleep(5)


    def get_name(self):
        name = ""
        # get the name from the user
        rospy.loginfo("Receptionist is getting name")
        i = 0
        # wait for the user to say something
        rospy.logwarn("Waiting for user input")
        self.say_publisher.publish(promts["ask_name"])
        time.sleep(5)
        rospy.logdebug("speech enable")
        self.speech_enable.publish(Bool(True))
        rospy.logdebug("speech enabled")
        rospy.logwarn("name1: " + name)
        rospy.logwarn(name  != "" and not rospy.is_shutdown())
        while not rospy.is_shutdown() and name == "":
            i+=1
            time.sleep(1)
            rospy.logdebug(name)
            if self.inputText != "":
                    name  = self.parser(calls["get_name"])
            if i%28 == 0:
                self.speech_enable.publish(Bool(False))
                self.say_publisher.publish(promts["ask_name"])
                time.sleep(4)
                self.speech_enable.publish(Bool(True))
                i = 0
                
        rospy.logwarn("name2: " + name)

        self.speech_enable.publish(Bool(False))
        

        rospy.logwarn(name)

        time.sleep(3)
        return name
    


    def is_someone(self):
        # check for a new detection in vision topic
        # check if any faces have been detected
        if self.faces is not None and len(self.faces) > 0:
            return True
        else:
            return False



    def parser(self, command):
        rospy.loginfo("Receptionist is parsing")
        rospy.logwarn(self.inputText)
        time.sleep(1)
        intent = ""
        try:
            rospy.logwarn(command + self.inputText)
            intent = self.callGPT(command + " " + self.inputText)
            # rospy,logwarn(intent)
            
            rospy.loginfo("Intent: " + intent)
            
        except:
            self.say_publisher.publish("I'm sorry, Could you rephrase?")
            rospy.logdebug("Failed response")
        rospy.logwarn(intent)
        
        return intent

            

    def input_callback(self, msg):
        self.inputText = msg.inputText
        rospy.logdebug("I heard: " + self.inputText)
            
    def callGPT(self, pr, t_max=256):
        rospy.logdebug("I am parsing in: GPT " + pr )

        response = openai.Completion.create(
            model=self.GPT_API.model,
            prompt=pr,
            max_tokens=t_max,
            n=1,
            temperature=self.GPT_API.temperature,
            frequency_penalty=self.self.GPT_API.frequency_penalty,
            presence_penalty=self.self.GPT_API.presence_penalty,    
        )
        rospy.logdebug("RESPONSE IS: response")
        return response.choices[0].text




def main():
    rospy.init_node('receptionist', anonymous=True)
    rospy.loginfo("Receptionist task created.")
    receptionist()


if __name__ == '__main__':
    main()