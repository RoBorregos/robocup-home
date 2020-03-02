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
    def __init__(self, filename):
        self.possible_actions = self.loadActions(filename)
        print(self.possible_actions)
        self.lastActionReceived = None
        # A typical pattern for entries is a tuple in the form: (priority_number, data).
        self.actionQueue = []
        self.waitingForConfirmation = False


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

    '''
        Callback generated when a new message is received from the action selectors
    '''
    def new_action_callback(self, msg):
        print("Action Queue:")
        print(self.actionQueue)
        print("A new action is triggered")
        print("intent:" + msg.intent)
        print("args: ")
        print(msg.args)
        if(self.possible_actions.get(msg.intent)):
            # registered action
            new_action = Action(msg.intent, self.possible_actions[msg.intent]['cmd_priority'],
                                self.possible_actions[msg.intent]['action_server_responsible'], msg.args)
            self.lastActionReceived = new_action
            if(self.decide_if_priority(new_action)):
                # Run new action
                
                # trigger new action
                self.trigger_new_action(new_action)
            else:
                # TODO: If same or lower priority ask the user what he wants to do
                print("Check for feedback of the user if needed")
                self.waitingForConfirmation = True
                print("New action has a lower priority than the one running")
                ##Notify the user
        else:
            print("Unrecognized action: " + msg.intent)
            ##Check if answer to confirmation?
            if(self.waitingForConfirmation):
                if(msg.intent=="yes" or msg.intent=="no"):
                    self.waitingForConfirmation = False
                    if(msg.intent=="yes"):
                        print("The user confirmed he prefers the new action over the previous one")
                        self.trigger_new_action(self.lastActionReceived)    
                else:
                    print("I still have a previous decision I must choose what to do?")
                 ##Remind the user you still have a decision to make on a previous action


    def trigger_new_action(self,new_action):
        print("New Action Triggered! Congrats")
        #Stop current Action!
        if(len(self.actionQueue)> 0):
            print("Stopping previous action...")
            current_action =  self.actionQueue[0]
        #Call the corresponding action server with the request
        #Add it at the front of the actionQueue
        self.actionQueue.insert(0,new_action)


    def decide_if_priority(self, new_action):
        #current_action.cancel_goal()
        if(len(self.actionQueue)==0):
            print("New action has a higher priority")
            return True
        current_action = self.actionQueue[0]
        if(new_action.priority > current_action.priority):
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
                     main_engine.new_action_callback)

    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    listener()
