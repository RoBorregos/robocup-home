#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String, Bool
from action_selectors.msg import RawInput

import os
import openai


promts = {
    "start": "Let's start carry my luggage task",
    "wait": "I am waiting for someone",
    "self_intro": "Hello, I'm homie, your personal assistant.",
    "point_to_bag" : "Which bag should I carry?"
    "right": "You said right, is that correct?",
    "left": "You said left, is that correct?",
    "grasp": "I will now pick your luggage"
    "follow" : "I am ready to follow you",
    "deliver" : "Please take your bag",
    "car" : "Is this the car?"
}

calls = {
    "get_name": "get me the name of the person on the next message:",
    "confirm" : "True or False, the next message is a confirmation"
}





INITIALIZATION_STATE = "init"
START_STATE = "start"
WAIT_STATE = "waiting for operator"
IDENTIFY_STATE = "identifying the bag"
GOING_STATE = "going somewhere"
DELIVER_STATE = "deliver bag"
RETURN_STATE = "returning to the arena"
END_STATE = "done"

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

        self.say_listener = rospy.Subscriber("sayer", Bool, self.say_callback)
        self.speech_enable.publish(Bool(False))

        #Nav subscribers 
        #self.cancel_move = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)
        #self.initial_pose = None
        #self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)
        self.is_moving = False

        rospy.loginfo("Carry-Luggage initialized")

        # Conversation dependencies
        
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=20)
        self.input_suscriber = rospy.Subscriber("RawInput", RawInput, self.input_callback)


        self.currentState = START_STATE
        # wait for the robot to be ready
        time.sleep(1)
        # start the main loop

        self.run()
        rospy.spin()


    def say_callback(self, data):
        self.saying = data.data
        rospy.logwarn("saying: " + str(self.saying))

    
    def run(self):
        rospy.loginfo("-----------------------------------------RUN-----------------------------------------")

        while not rospy.is_shutdown():
            rospy.loginfo("-----------------------------------------RUN-----------------------------------------")

            if self.currentState == START_STATE:
                self.speech_enable.publish(Bool(False))

                rospy.loginfo("Carrier is ready to start")
                self.say_publisher.publish(promts["start"])
                time.sleep(1)

                rospy.logwarn("here it should end the speech") 
                i = 0
                self.currentState = WAIT_STATE

                
            elif self.currentState == WAIT_STATE:
                i += 1
                if self.is_someone():
                    self.say_publisher.publish(promts["self_intro"])
                    self.currentState = IDENTIFY_STATE
                else:
                    #if there is no one, wait for 3 seconds and check again
                    time.sleep(3)

                    if i%10:
                        self.say_publisher.publish(promts["wait"])
                        i = 0
                        

                
            elif self.currentState == IDENTIFY_STATE:
                grasped = self.get_bag()
                if grasped != "":
                    self.currentState = GOING_STATE
                else:
                    self.currentState = IDENTIFY_STATE       

            elif self.currentState == GOING_STATE:
                rospy.loginfo("Receptionist is going to the person")
    
    def get_bag(self):
        grasped = ""
        rospy.logwarn("Waiting for operator")
        self.say_publisher.publish(promts["point_to_bag"])
        #como poner aqui algo, como suscribirse a un nodo
        #que le indique a donde apunto
        flag = ""
        if flag == "right":
            self.say_publisher.publish(promts["right"])
        elif flag == "left":
            self.say_publisher.publish(promts["left"])
        else: #flag == ""
            return
        time.sleep(5)
        rospy.logdebug("speech enabled")
        self.speech_enable.publish(Bool(True))
        confirm_ = self.callGPT(calls["confirm"] + ": " + self.inputText)
        if confirm_ == "False":
            #settear la variable al lado de la bag correcto
        #Indicar que ya tomara la bag
        self.say_publisher.publish(promts["grasp"])
        #aqui tiene que saber que ya lo tomo
        self.say_publisher.publish(promts["follow"])
        
        #Falta el #3 de calls[confirm] para saber si si llegamos
        self.say_publisher.publish(promts["car"])
        
        #4
        self.say_publisher.publish(promts["deliver"])
            
        

    def is_someone(self):
        check for a new detection in vision topic
        # rospy.loginfo("checking if someone is there")
        # time.sleep(5)
        return True

            

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
    rospy.init_node('carrybag', anonymous=True)
    rospy.loginfo("Carry my Luggage task created.")
    receptionist()


if __name__ == '__main__':
    main()