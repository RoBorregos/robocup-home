#!/usr/bin/env python
from main_engine.action_manager import *
import os
import unittest
import sys
PKG = 'main_engine'


# Testing the Main engine module of the robot
"""
    action_selector_msg -> 
       Header header
       string intent
       string[] args
"""
class ActionSelectorMsg(object):
    def __init__(self,intent,action_client_binded, args):
        self.intent=intent
        self.action_client_binded = action_client_binded
        self.args = args


class TestActionManager(unittest.TestCase):
    def setUp(self):
        self.action_manager = Main_Engine()

        # Mock message go to the kitchen
        self.go_to_msg= ActionSelectorMsg("go_to", "navServer", ["kitchen"])
        # Mock message pick up un juguito
        self.pick_up_msg = ActionSelectorMsg("pick_up", "armServer", ["juguito"])
     
    def test_initial_values(self):
        # verifies action queue starts empty
        self.assertEqual(self.action_manager.action_queue, [], "Not empty")

    def test_create_action(self):
        # Creates new action into the action queue
        self.action_manager.new_action_request_callback(self.go_to_msg)
        feedback = self.action_manager.action_queue[0].get_feedback()
        self.assertGreater(len(self.action_manager.action_queue),0)
        self.assertEqual(self.action_manager.action_queue[0].id, "go_to")
        self.assertEqual(feedback,"PENDING")

    def test_create_multiple_actions(self):
        self.action_manager.new_action_request_callback(self.go_to_msg)
        self.action_manager.new_action_request_callback(self.pick_up_msg)
        
        self.assertEqual(len(self.action_manager.action_queue),2)

    def test_remove_action(self):
        self.action_manager.new_action_request_callback(self.go_to_msg)
        self.action_manager.delete_an_action(0)

        self.assertEqual(len(self.action_manager.action_queue),0)
        self.assertEqual(len(self.action_manager.history),1)
        self.assertEqual(self.action_manager.history[0].id, self.go_to_msg.intent)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_action_manager', TestActionManager)
