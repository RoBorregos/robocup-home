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
CATEGORIES = 2


class Main_Engine(object):
    def __init__(self, filename):
        self.possible_actions = self.loadActions(filename)
        print(self.possible_actions)
        self.lastActionReceived = None
        # TODO: Multidimensional for n categories defined
        self.action_queue = [ [] for i in range(CATEGORIES) ]
        

    def loadActions(self, filename):
        '''
        Returns a dictionary with the possible actions defined in the csv file
        {
            cmd_id:{cmd_category,cmd_priority,require_args}
        }
        '''
        directory = os.path.dirname(os.path.realpath(__file__))
        absolute_path_to_csv = os.path.join(directory, filename)
        print("Opening possible actions: "+absolute_path_to_csv)
        dictionary_possible_actions = dict()
        with open(absolute_path_to_csv, 'r') as action_file:
            dict_reader = csv.DictReader(action_file)
            for row in dict_reader:
                cmd_id = row.get('cmd_id')
                del row['cmd_id']
                dictionary_possible_actions[cmd_id] = dict(row)
        return dictionary_possible_actions


    def shutdown_callback(self):
        print("CRITICAL -------------Shutting down main_engine--------------")


    def add_to_priority_queue(self,new_action,action_queueCategory):
        for index, action in enumerate(action_queueCategory):
            ##Priority to newer commands over old ones
            if(action.priority>=new_action.priority):
                action_queueCategory.insert(index,new_action)
                return action_queueCategory
        action_queueCategory.append(new_action)
        return action_queueCategory


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
        if(self.possible_actions.get(msg.intent)):
            # registered action
            new_action = Action(msg.intent, self.possible_actions[msg.intent]['cmd_priority'],
                                self.possible_actions[msg.intent]['action_client_binded'], self.possible_actions[msg.intent]['cmd_category'], msg.args)
            self.lastActionReceived = new_action
            print(new_action.category)           
            ##Stop Action
            if(new_action.id=="stop"):
                self.stop_everything()
                return
            
            action_queueOfCategory = self.action_queue[new_action.category]
            # Check what to do
            if(self.decide_if_priority(new_action)):
                # Run new action
                self.trigger_new_action(new_action)
            #Add it to the queue (doesn't matter if it was triggered or not)
            self.action_queue[new_action.category] = self.add_to_priority_queue(new_action,action_queueOfCategory)
        else:
            print("Action requested not found in action db")
    

    def printaction_queue(self):
        for index,category in enumerate(self.action_queue):
            print("*********************")
            print("Category "+str(index))
            for action in category:
                action.print_self(endline=False)
            print("*********************")
 

    def stop_everything(self):
        #Stop every action_server (as fast as possible!)
        print("Stop requested")
        
            
    def trigger_new_action(self, new_action):
        print("New Action Triggered! Congrats")
        # Stop current Action!
        if(len(self.action_queue[new_action.category]) > 0):
            print("Stopping previous action...")
            # Find the first ocurrence of the same category
            current_action = self.action_queue[new_action.category][0]
            # If previous action done (current_action.feedback == DONE|FAILED) remove from queue
            current_action.stop()
            # Else keep it in the queue
        # Call the corresponding action server with the request
        new_action.run()

    def decide_if_priority(self, new_action):
        # WARNING: NEW ACTIONS HAVE PRIORITY OVER OLD ONES
        if(len(self.action_queue[new_action.category]) == 0):
            print("New action has a higher priority")
            return True
        current_action = self.action_queue[new_action.category][0]
        if(new_action.priority >= current_action.priority):
            print("New action has a higher priority")
            return True
        print("New action has a lower priority")
        return False


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    FILENAME_OF_CSV = 'possible_actions.csv'
    main_engine = Main_Engine(FILENAME_OF_CSV)

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
