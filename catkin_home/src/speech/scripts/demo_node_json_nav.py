#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import current_thread
import rospy
import actionlib

import json

from std_msgs.msg import String, Int32, Bool

from arm_server.msg import MoveArmAction, MoveArmGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

# from main_eng.srv import MainEng, MainEngResponse
from speech.msg import command, list_of_commands
import uuid

COMMANDS_TOPIC = "/speech/processed_commands"
MAN_SERV_TOPIC = "/manipulation_server"
TRACKING_TOPIC = "/toggle_face_detection"
MOVE_ARM_AS = '/move_arm_as'

JSON_PATH = "maps/tdp_locations.json"

NAV_ENABLED = True
MANIP_ENABLED = False
SAY_ENABLED = False
TRACKING_ENABLED = False

OBJECTS_NAME= {
    1 : 'COKE',
    2 : 'APPLE',
    3 : 'MUG',
    4 : 'SOAP',
    5 : 'BANANA',
    6 : 'JAR'
}

OBJECTS_ID= {
    'COKE' : 1,
    'APPLE' : 2,
    'MUG' : 3,
    'SOAP' : 4,
    'BANANA' : 5,
    'JAR' : 6
}

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
        self.nav_json = json.load(open(JSON_PATH))
        # print json
        print(self.nav_json)
        # self.nav_client = actionlib.SimpleActionClient('navServer', main_eng.msg.navServAction)
        # self.nav_client.wait_for_server()
        # self.manip_client = actionlib.SimpleActionClient(MAN_SERV_TOPIC, command) #CHANGE THIS
        # self.manip_client.wait_for_server()

        # for pose goals
        rospy.loginfo("Initializing main_eng_node")
        rospy.loginfo(f"NAV_ENABLED: {NAV_ENABLED}")
        rospy.loginfo(f"MANIP_ENABLED: {MANIP_ENABLED}")
        rospy.loginfo(f"SAY_ENABLED: {SAY_ENABLED}")
        rospy.loginfo(f"TRACKING_ENABLED: {TRACKING_ENABLED}")

        if NAV_ENABLED:
            rospy.loginfo("Connecting to nav_server")
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.move_base_client.wait_for_server()
            rospy.loginfo("Connected to nav_server")
        if MANIP_ENABLED:
            self.manipulation_pub = rospy.Publisher('manipulation/goal', String, queue_size=10)
            self.manipulation_sub = rospy.Subscriber('manipulation/response', String, self.manipulation_callback)
            rospy.loginfo("Connecting to manipulation_server")
            self.move_arm_client = actionlib.SimpleActionClient(MOVE_ARM_AS, MoveArmAction)
            self.move_arm_client.wait_for_server()
            rospy.loginfo("Connected to manipulation_server")

            # self.reset_arm()

        rospy.loginfo("MainEngServer started")
        self.state = MainEngServer.STATE_ENUM["IDLE"]
        if TRACKING_ENABLED:
            self.tracker_pub = rospy.Publisher(TRACKING_TOPIC, Int32, queue_size=10)
            self.tracker_pub.publish(1)
        self.current_command = None
        self.current_queue = []
        self.current_thread = None

        rospy.spin()

    def reset_arm(self):
        self.arm_goal = MoveArmGoal()
        self.arm_goal.state = "hri"
        self.arm_goal.speed = 0.3

        #self.move_arm_client.send_goal(self.arm_goal)
        #self.move_arm_client.wait_for_result()
    
    def manipulation_callback(self, data):
        self.manipulation_status = data.data

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
            if NAV_ENABLED:
                self.move_base_client.cancel_all_goals()
            # self.nav_client.cancel_goal()
        elif self.current_command == "grab":
            rospy.loginfo("Cancelling grab")
            if MANIP_ENABLED:
                self.move_arm_client.cancel_all_goals()
            # self.manip_client.cancel_goal()
            pass
        elif self.current_command == "put":
            rospy.loginfo("Cancelling put")
            if MANIP_ENABLED:
                self.move_arm_client.cancel_all_goals()
            # self.manip_client.cancel_goal()
            pass
        elif self.current_command == "introduce":
            rospy.loginfo("Cancelling introduce")
            # self.cancel_introduce()
            pass
        else:
            rospy.logwarn("Unknown command")

    def main_eng_command_subscriber(self, commands_input):
        rospy.loginfo("Received command")

        if not commands_input:
            return MainEngServer.STATE_ENUM["ERROR"]

        # current_thread = uuid.uuid1()
        # self.current_thread = current_thread

        if self.state != MainEngServer.STATE_ENUM["IDLE"]:
            rospy.logwarn("Received command while not in IDLE state, cancelling current commands")
            #return MainEngServer.STATE_ENUM["ERROR"]
            self.current_queue = []
            self.cancel_current_command()
            self.state = MainEngServer.STATE_ENUM["IDLE"]

        current_thread = uuid.uuid1()
        self.current_thread = current_thread
        if TRACKING_ENABLED:
            self.tracker_pub.publish(0)
        self.state = MainEngServer.STATE_ENUM["RECEIVE_COMMANDS"]
        # self.current_queue = commands_input.data.split(", ")
        self.current_queue = commands_input.commands
        self.past_state = self.state
        self.current_place = rospy.Subscriber("/robot_pose", Pose)
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
        # self.reset_arm()
        if TRACKING_ENABLED:
            self.tracker_pub.publish(1)
        if self.state == MainEngServer.STATE_ENUM["IDLE"]:
            self.current_thread = None
        if  MANIP_ENABLED:
            self.move_arm_client.cancel_all_goals()
        # self.move_base_client.cancel_all_goals()
        # self.tracker_pub.publish(1)
        # self.main_eng_command_manager()


    def main_eng_command_manager(self, command): 
        # commands = message_input.split(", ")
        # if not commands or len(commands) == 0:
        #     return MainEngServer.STATE_ENUM["ERROR"]
        # for command in commands:
            # action, value = command.split(" ")
        action = command.action
        value = command.complements
        rospy.loginfo("Action: " + action + " Value: ")
        if(len(value) == 0):
            value = None
            self.state = MainEngServer.STATE_ENUM["ERROR"]
            return MainEngServer.STATE_ENUM["ERROR"]
        value = value[0]
        rospy.loginfo("Action: " + action + " Value: " + value)
        if action == "go":
            self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
            if NAV_ENABLED:
                self.go_to_past_location()
        elif action == "stop":
            self.state = MainEngServer.STATE_ENUM["STOPPING_SERVICES"]
            self.stop()
        elif action == "grab":
            self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
            if MANIP_ENABLED:
                self.grab(value)
        elif action == "put":
            self.state = MainEngServer.STATE_ENUM["EXECUTING_COMMANDS"]
            if value == "user":
                if NAV_ENABLED:
                    self.go_to_past_location()
                return
            if NAV_ENABLED:
                self.go_to_location(value)
            if MANIP_ENABLED:
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

    def go_to_past_location(self):
        rospy.loginfo("Going to past location")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.past_location
        self.move_base_client.send_goal_and_wait(goal)

    def go_to_location(self, location):
        KEY_DICT = {
            "outside" : "outside_safe",
            "living_room": "living_room_safe",
            "living_room_table": "living_room_table",
            "living_room_side_table": "living_room_side_table",
            "kitchen_table": "kitchen_table",
            "kitchen_side_table": "kitchen_side_table",
            "hallway": "hallway_safe_out"
        }
        if location in KEY_DICT.values():
            location = location
        elif location in KEY_DICT.keys():
            location = KEY_DICT[location]
        else:
            rospy.logwarn(f"Location not found: {location}")
            return False
        for key in self.nav_json:
            if key.lower() == location.lower():
                rospy.logwarn(f"Sending location: {location}")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.nav_json[key][0]
                goal.target_pose.pose.position.y = self.nav_json[key][1]
                goal.target_pose.pose.position.z = self.nav_json[key][2]
                goal.target_pose.pose.orientation.x = self.nav_json[key][3]
                goal.target_pose.pose.orientation.y = self.nav_json[key][4]
                goal.target_pose.pose.orientation.z = self.nav_json[key][5]
                goal.target_pose.pose.orientation.w = self.nav_json[key][6]

                self.move_base_client.send_goal(goal)
                self.move_base_client.wait_for_result()
                rospy.logdebug(f"Result: {self.move_base_client.get_result()}")
                return True
        rospy.logwarn(f"Location not found: {location}")
        return False


    def stop(self):
        rospy.loginfo("Stopping")

    def grab(self, value):
        MODEL_DICT = {
            "Apple": "manzanas",
            "Cookies": "galletas",
            "Pringles": "pringles",
            "Coke": "cocacola",
            "Cereal": "zucaritas",
            "soap": "zote"
        }
        value = MODEL_DICT[value]
        self.manipulation_pub.publish(value)
        rospy.loginfo("Waiting for grab")
        status = self.manipulation_sub = rospy.Subscriber('manipulation/response', String, self.manipulation_callback)
        rospy.loginfo("Grabbing " + value)
        if status == "True":
            rospy.loginfo("Grabbed " + value)
            # self.say(promts["grasp_success"])
            return True
        elif status == "False":
            rospy.loginfo("Failed to grab " + value)
            # self.say(promts["grasp_fail"])
            return False

    def put(self, value):
        rospy.loginfo("Putting " + value)
        if(value == "user"):
            self.cancel_current_command()
            self.current_queue = []
            self.move_base_client.cancel_all_goals()
            self.state = MainEngServer.STATE_ENUM["IDLE"]
            return
        self.manipulation_pub.publish("Place")
        status = self.manipulation_sub = rospy.Subscriber('manipulation/response', String, self.manipulation_callback)
        rospy.loginfo("Waiting for put")
        if status == "True":
            rospy.loginfo("Put " + value)
            # self.say(promts["put_success"])
            return True
        elif status == "False":
            rospy.loginfo("Failed to put " + value)
            # self.say(promts["put_fail"])
            return False

    def intro(self):
        rospy.loginfo("Introducing")

if __name__ == "__main__":
    try:
        MainEngServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
        pass