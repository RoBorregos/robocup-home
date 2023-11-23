#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import current_thread
import rospy
import actionlib

from std_msgs.msg import String, Int32

# from main_eng.srv import MainEng, MainEngResponse
from speech.msg import command, list_of_commands
import uuid

COMMANDS_TOPIC = "/speech/processed_commands"
MAN_SERV_TOPIC = "/manipulation_server"
TRACKING_TOPIC = "/tracking_server"


class MainEngServer:
    state = 0
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 10,
        "EXECUTING_COMMANDS": 20,
        "STOPPING_SERVICES": 300,
        "ERROR": 500,
        "SHUTDOWN": 1000
    }
    COMMANDS_ENUM = {
        "go": 0,
        "stop": 10,
        "grab": 20,
        "put": 30,
    }
    def __init__(self):
        self.node = rospy.init_node('main_eng_server')
        # self.pub = rospy.Service('main_eng', MainEng, self.main_eng_server)
        self.sub = rospy.Subscriber(COMMANDS_TOPIC, list_of_commands, self.main_eng_command_subscriber) 
        self.rate = rospy.Rate(10)
        rospy.loginfo("Connecting to nav_server")
        # self.nav_client = actionlib.SimpleActionClient('navServer', main_eng.msg.navServAction)
        # self.nav_client.wait_for_server()
        self.manip_client = actionlib.SimpleActionClient(MAN_SERV_TOPIC, command) #CHANGE THIS
        self.manip_client.wait_for_server()
        rospy.loginfo("MainEngServer started")
        self.state = MainEngServer.STATE_ENUM["IDLE"]
        self.tracker_pub = rospy.Publisher(TRACKING_TOPIC, Int32, queue_size=10)
        self.tracker_pub.publish(1)
        self.current_command = None
        self.current_queue = []
        self.current_thread = None

        rospy.spin()

    # def main_eng_server(self, req):
    #     message_input:str = req.message
    #     if not message_input:
    #         return MainEngResponse(MainEngServer.STATE_ENUM["ERROR"])
        
    #     if self.state != MainEngServer.STATE_ENUM["IDLE"]:
    #         rospy.logwarn("Received command while not in IDLE state")
    #         # if stoping services, wait for services to stop then override command
    #     else:
    #         self.state = MainEngServer.STATE_ENUM["RECEIVE_COMMANDS"]
    #         return self.main_eng_command_manager(message_input)

    def cancel_current_command(self):
        rospy.loginfo("Cancelling current command")
        if self.current_command == "go":
            rospy.loginfo("Cancelling go")
            # self.nav_client.cancel_goal()
        elif self.current_command == "grab":
            rospy.loginfo("Cancelling grab")
            # self.manip_client.cancel_goal()
            pass
        elif self.current_command == "put":
            rospy.loginfo("Cancelling put")
            # self.manip_client.cancel_goal()
            pass
        elif self.current_command == "introduce":
            rospy.loginfo("Cancelling introduce")
            # self.cancel_introduce()
            pass
        else:
            rospy.logwarn("Unknown command")

    def main_eng_command_subscriber(self, commands_input):
        if not commands_input:
            return MainEngServer.STATE_ENUM["ERROR"]

        # current_thread = uuid.uuid1()
        # self.current_thread = current_thread

        if self.state != MainEngServer.STATE_ENUM["IDLE"]:
            rospy.logwarn("Received command while not in IDLE state, cancelling current commands")
            return MainEngServer.STATE_ENUM["ERROR"]
            self.current_queue = []
            self.cancel_current_command()
            self.state = MainEngServer.STATE_ENUM["IDLE"]

        current_thread = uuid.uuid1()
        self.current_thread = current_thread
        self.tracker_pub.publish(0)
        self.state = MainEngServer.STATE_ENUM["RECEIVE_COMMANDS"]
        # self.current_queue = commands_input.data.split(", ")
        self.current_queue = commands_input.commands
        self.past_state = self.state
        while len(self.current_queue) > 0 and self.current_thread == current_thread:
            self.current_command = self.current_queue.pop(0)
            self.past_state = self.main_eng_command_manager(self.current_command)
            if self.past_state == MainEngServer.STATE_ENUM["ERROR"]:
                rospy.logerror("Error in command manager")
                self.state = MainEngServer.STATE_ENUM["ERROR"]
                self.cancel_current_command()
                self.current_queue = []
                self.state = MainEngServer.STATE_ENUM["IDLE"]
                break
        self.tracker_pub.publish(1)
        # self.main_eng_command_manager()


    def main_eng_command_manager(self, commands: list[command]): 
        # commands = message_input.split(", ")
        if not commands or len(commands) == 0:
            return MainEngServer.STATE_ENUM["ERROR"]
        for command in commands:
            # action, value = command.split(" ")
            action = command.action
            value = command.value[1]
            if action == "go":
                self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
                self.go(value)
            elif action == "stop":
                self.state = MainEngServer.STATE_ENUM["STOPPING_SERVICES"]
                self.stop()
            elif action == "grab":
                self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
                self.grab(value)
            elif action == "put":
                self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
                self.go(value)
                self.put(value)
            elif action == "introduce":
                self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
                self.intro()
            else:
                return MainEngServer.STATE_ENUM["ERROR"]
        
        self.state = MainEngServer.STATE_ENUM["IDLE"]
        return MainEngServer.STATE_ENUM["IDLE"]

    def go(self, value):
        rospy.loginfo("Going to " + value)
        # self.nav_client.send_goal_and_wait(target_location = value)
        rospy.loginfo("Arrived at " + value)

    def stop(self):
        rospy.loginfo("Stopping")

    def grab(self, value):
        rospy.loginfo("Grabbing " + value)

    def put(self, value):
        rospy.loginfo("Putting ", value)

    def intro(self):
        rospy.loginfo("Introducing")

if __name__ == "__main__":
    try:
        MainEngServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass