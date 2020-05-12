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
PREVIOUS_ACTION_NOT_DONE = False
PREVIOUS_ACTION_DONE = True

class Main_Engine(object):
    def __init__(self):
        self.lastActionReceived = None
        # TODO: Multidimensional for n categories defined
        self.action_queue = []
        self.history = []


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
            self.trigger_new_action(new_action,PREVIOUS_ACTION_NOT_DONE)
        #Add it to the queue (doesn't matter if it was triggered or not)
        self.action_queue.append(new_action)
        print("new action added to the queue")
     

    def print_action_queue(self):
            print("*********ACTION QUEUE************")
            for action in self.action_queue:
                action.print_self(endline=False)
            print("*********************************")
            print("")
 

    def stop_everything(self):
        #Stop every action_server (as fast as possible!)
        print("Stop requested")
        
            
    def trigger_new_action(self, new_action,previous_action_done):
        print("New Action Triggered! Congrats")
        # Stop current Action!
        if(len(self.action_queue) > 0):
            print("Stopping previous action...")
            # Find the first ocurrence of the same category
            if(previous_action_done):
                self.delete_an_action(0)
            # Else keep it in the queue       
        # Call the corresponding action server with the request
        new_action.run()

    def delete_an_action(self,position_in_queue):
        #current_action = self.action_queue[position_in_queue]
        #current_action.stop()
        self.history.append(self.action_queue.pop(position_in_queue))


    '''
    monitor(): Checks the status of the current action being executed, deletes it from the queue
    when done, notifies if an error arises.
    '''
    def monitor(self):
        if(len(self.action_queue) > 0):
            current_action = self.action_queue[0]
            # Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
            current_action_status  = current_action.get_feedback()
            if(current_action_status == "SUCCEEDED"):
                current_action.print_self()
                print('Action Completed Succesfully!')
                if(len(self.action_queue)>1):
                    #There are more actions pending
                    self.trigger_new_action(self.action_queue[1],PREVIOUS_ACTION_DONE)
                else:
                    self.delete_an_action(0)
            elif(current_action_status == "LOST"):
                current_action.print_self()
                print('Action Failed!')
                self.delete_an_action(0)

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
    while not rospy.is_shutdown():
        main_engine.print_action_queue()
        main_engine.monitor()
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    listener()
