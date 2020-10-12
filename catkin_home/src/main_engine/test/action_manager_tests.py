#!/usr/bin/env python
from main_engine.action_manager import Main_Engine
import os
import unittest
import sys
PKG = 'main_engine'


# Testing the Main engine module of the robot

class TestActionManager(unittest.TestCase):
    def setUp(self):
        self.action_manager = Main_Engine()

    def test_initial_values(self):
        self.assertEquals(self.action_manager.action_queue, [], "Not empty")


if __name__ == '__main__':
    import rosunit
    rosrunit.unitrun(PKG, 'test_action_manager', TestActionManager)
