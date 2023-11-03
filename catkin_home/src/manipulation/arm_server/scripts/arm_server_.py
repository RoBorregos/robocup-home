#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import moveit_commander
from sensor_msgs.msg import JointState
from arm_server.msg import MoveArmAction, MoveArmResult, MoveArmFeedback, MoveArmGoal
from arm_server.srv import Gripper, GripperResponse


class ArmServer:

    def __init__(self):
        self.ARM_GROUP = rospy.get_param("ARM_GROUP", "arm")
        self.ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.GRIPPER_GROUP = rospy.get_param("GRIPPER_GROUP", "gripper")
        self.HEAD_GROUP = rospy.get_param("HEAD_GROUP", "head")

        # ARM GROUP STATES
        self.ARM_HOME = [0.0, 0.0, 0.0, -1.5708, 1.5708, 0.7854]
        self.ARM_CALIBRATION = [-1.57, 0.0, -3.1416 / 4, 0, -3.1416 / 4, -2.356]
        self.ARM_NAV = [-1.5708, -1.0472, -1.0472, 1.5708, 0.0, -0.7854]
        self.ARM_HRI = [-1.5708, -1.0472, -1.0472, 1.5708, 0.0, -0.7854]
        rospy.init_node('arm_server')
        
        self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP, wait_for_servers = 0)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP, wait_for_servers = 0)
        
        #self.move_joints(self.ARM_HOME)
        self.move_arm_as = actionlib.SimpleActionServer(
            "/move_arm_as", MoveArmAction, execute_cb=self.move_arm_cb, auto_start=False
        )
        self.move_arm_as.start()
        self.gripper_server = rospy.Service("/gripper_service", Gripper, self.handle_gripper)
        rospy.spin()

    def move_arm_cb(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))
        if goal.state != None:
            if(goal.state == "home"):
                goal.joints_target = self.ARM_HOME
            elif(goal.state == "calibration"):
                goal.joints_target = self.ARM_CALIBRATION
            elif(goal.state == "nav"):
                goal.joints_target = self.ARM_NAV
            elif(goal.state == "hri"):
                goal.joints_target = self.ARM_HRI

        self.move_joints(goal.joints_target, goal.speed)
        result = MoveArmResult()
        result.success = 1
        self.move_arm_as.set_succeeded(MoveArmResult())

    def move_joints(self, joint_values, speed):
        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = joint_values
        # set speed
        self.arm_group.set_max_velocity_scaling_factor(speed)
        self.arm_group.go(joint_state, wait=True)
        self.arm_group.stop()

    def handle_gripper(self, req):
        g_state = req.state
        rospy.loginfo("Executing gripper state: {}".format(g_state))
        self.gripper_group.set_named_target( g_state )
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        rospy.sleep(0.25)
        return GripperResponse(success=True)
   
if __name__ == '__main__':
    ArmServer()
