#!/usr/bin/env python

# DESCRIPTION
from intercom.msg import action_selector_cmd
import time
from threading import Thread
import rospy
import csv
import os
from Action import Action

"""
	TODO: Define description
"""

# TOPICS PUBLISHED
"""
    NA
"""

# TOPICS READ
"""
    engine_commands -> String
"""

# MSG USED
"""
    action_selector_msg -> 
       Header header
       string intent
       string[] args
"""


class Main_Engine(object):
    def __init__(self):
        self.lastActionReceived = None
        # TODO: Multidimensional for n categories defined
        self.action_queue = []


    def shutdown_callback(self):
        self.stop_everything()
        print("CRITICAL -------------Shutting down main_engine--------------")


    '''
        Callback generated when a new message is received from the action selectors
    '''
    def new_action_request_callback(self, msg):
        print("Action Queue:")
        print(self.action_queue)
        print("A new action is requested")
        print("intent:" + msg.intent)
        print("args: ")
        print(msg.args)
        # registered action
        new_action = Action(msg.intent,
                            msg.action_client_binded, msg.args)
        self.lastActionReceived = new_action       
        ##Stop Action
        if(new_action.id=="stop"):
            self.stop_everything()
            return
        if(len(self.action_queue)==0):
            self.trigger_new_action(new_action)
        #Add it to the queue (doesn't matter if it was triggered or not)
        self.action_queue.append(new_action)
     

    def printaction_queue(self):
            print("*********************")
            for action in self.action_queue:
                action.print_self(endline=False)
            print("*********************")
 

    def stop_everything(self):
        #Stop every action_server (as fast as possible!)
        print("Stop requested")
        
            
    def trigger_new_action(self, new_action):
        print("New Action Triggered! Congrats")
        # Stop current Action!
        if(len(self.action_queue) > 0):
            print("Stopping previous action...")
            # Find the first ocurrence of the same category
            current_action = self.action_queue[0]
            # If previous action done (current_action.feedback == DONE|FAILED) remove from queue
            current_action.stop()
            # Else keep it in the queue       
        # Call the corresponding action server with the request
        new_action.run()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    main_engine = Main_Engine()

    rospy.init_node('action_manager', anonymous=False)
    rospy.on_shutdown(main_engine.shutdown_callback)
    rospy.Subscriber("action_requested", action_selector_cmd,
                     main_engine.new_action_request_callback)
    rate = rospy.Rate(0.5) # 5hz
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        main_engine.printaction_queue()
        print("Spin!")
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    listener()
