#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import time
import moveit_commander
from sensor_msgs.msg import JointState
from arm_server.msg import MoveArmAction, MoveArmResult, MoveArmFeedback, MoveArmGoal
from arm_server.srv import Gripper, GripperResponse
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Bool


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
        rospy.Subscriber("/stop_arm", Bool, self.stop_arm_cb)
        self.stop_arm = False
        
        #self.move_joints(self.ARM_HOME)
        self.arm_as = actionlib.SimpleActionServer(
            "/arm_as", MoveArmAction, execute_cb=self.arm_cb, auto_start=False,
        )
        self.arm_as.start()
        self.arm_as_feedback = MoveArmFeedback()
        self.arm_as_result = MoveArmResult()
        self.gripper_server = rospy.Service("/gripper_service", Gripper, self.handle_gripper)
        rospy.spin()

    def stop_arm_cb(self, msg):
        if msg.data == True:
            self.stop_arm = True
            self.arm_group.stop()

    def arm_cb(self, goal):
        rospy.loginfo("Received goal: {}".format(goal))
        init_t = time.time()
        result = MoveArmResult()

        if goal.speed == None:
            goal.speed = 0.15
        if goal.acceleration == None:
            goal.acceleration = 0.025
        if goal.position_tolerance == None:
            goal.position_tolerance = 0.012
        if goal.orientation_tolerance == None:
            goal.orientation_tolerance = 5

        self.arm_group.set_max_velocity_scaling_factor(goal.speed)
        self.arm_group.set_max_acceleration_scaling_factor(goal.acceleration)
        self.arm_group.set_goal_position_tolerance(goal.position_tolerance)
        self.arm_group.set_goal_orientation_tolerance( np.deg2rad(goal.orientation_tolerance) )

        if goal.pose_target != None:
            result.success = self.plan_arm(goal.pose_target, goal.planning_time, goal.planning_attempts)
        
        else:
            if goal.state != None:
                if(goal.state == "home"):
                    goal.joints_target = self.ARM_HOME
                elif(goal.state == "calibration"):
                    goal.joints_target = self.ARM_CALIBRATION
                elif(goal.state == "nav"):
                    goal.joints_target = self.ARM_NAV
                elif(goal.state == "hri"):
                    goal.joints_target = self.ARM_HRI

            result.success = self.move_joints(goal.joints_target, goal.speed)

        self.arm_as.set_succeeded( result )

    def move_joints(self, joint_values, speed):
        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = joint_values
        # set speed
        self.set_pose_target(None)
        self.arm_group.set_max_velocity_scaling_factor(speed)
        self.arm_group.go(joint_state)
        
        feedback.execition_state = "Moving"

        self.arm_as.publish_feedback( feedback )
        t = time.time()
        group.go()
        current_joints = self.arm_group.get_current_joint_values()
        distance = np.sqrt( (joint_values[0] - current_joints[0])**2 + (joint_values[1] - current_joints[1])**2 + (joint_values[2] - current_joints[2])**2 + (joint_values[3] - current_joints[3])**2 + (joint_values[4] - current_joints[4])**2 + (joint_values[5] - current_joints[5])**2 )
        max_distance = distance

        while( not self.stop_arm or self.arm_as.is_preempt_requested() or distance > 0.1 ):
            if( self.stop_arm or self.arm_as.is_preempt_requested() ):
                rospy.loginfo("Stopping arm")
                self.arm_group.stop()
                self.arm_as.set_preempted()
                return False
            current_joints = self.arm_group.get_current_pose().pose
            distance = np.sqrt( (joint_values[0] - current_joints[0])**2 + (joint_values[1] - current_joints[1])**2 + (joint_values[2] - current_joints[2])**2 + (joint_values[3] - current_joints[3])**2 + (joint_values[4] - current_joints[4])**2 + (joint_values[5] - current_joints[5])**2 )
            feedback.complete_percentage = 1 - distance / max_distance
            self.arm_as.publish_feedback( feedback )
            rospy.sleep(0.1)

        rospy.loginfo("Execution time: " + str(time.time() - t))
        self.arm_group.stop()
        return True        

    def plan_arm(self, pose, p_time, p_attempts):
        feedback = MoveArmFeedback()

        planners = ["ompl"]
        for planner in planners:
            self.arm_group.set_planning_pipeline_id(planner)
            self.arm_group.set_planner_id("RRTConnect")
            self.arm_group.set_planning_time(p_time)
            self.arm_group.set_num_planning_attempts(p_attempts)
            self.forget_joint_values()
            self.set_pose_target(pose)
            
            print("Planning to pose: " + str(pose_st.pose))
            feedback.execition_state = "Planning"
            self.arm_as.publish_feedback( feedback )
                
            t = time.time()
            res = group.plan()
            rospy.loginfo("Planning time: " + str(time.time() - t))
            if res[0] == False:
                rospy.loginfo("Planning failed")
                return false

        feedback.execition_state = "Moving"
        self.arm_as.publish_feedback( feedback )
        t = time.time()
        group.go()
        current_pose = self.arm_group.get_current_pose().pose
        pose_distance = np.sqrt( (pose.position.x - current_pose.position.x)**2 + (pose.position.y - current_pose.position.y)**2 + (pose.position.z - current_pose.position.z)**2 )
        max_distance = pose_distance

        while( not self.stop_arm or self.arm_as.is_preempt_requested() or pose_distance > 0.1 ):
            if( self.stop_arm or self.arm_as.is_preempt_requested() ):
                rospy.loginfo("Stopping arm")
                self.arm_group.stop()
                self.arm_as.set_preempted()
                return False
            current_pose = self.arm_group.get_current_pose().pose
            pose_distance = np.sqrt( (pose.position.x - current_pose.position.x)**2 + (pose.position.y - current_pose.position.y)**2 + (pose.position.z - current_pose.position.z)**2 )
            feedback.complete_percentage = 1 - pose_distance / max_distance
            self.arm_as.publish_feedback( feedback )
            rospy.sleep(0.1)

        rospy.loginfo("Execution time: " + str(time.time() - t))
        self.arm_group.stop()
        return True        
        

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
