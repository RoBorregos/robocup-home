#! /usr/bin/env python3

import json
import math
import tf

import numpy
import pathlib
import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actions.msg
from geometry_msgs.msg import Twist 
from actions.msg import manipulationServAction, manipulationServGoal, manipulationServResult


class MoveItErrorCodes(Enum):
    SUCCESS = 1
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7

BASE_PATH = str(pathlib.Path(__file__).parent) + '/../../../../'


OBJECTS_NAME= {
    1 : 'VEGETABLES',
    2: 'TEA',
    3 : 'COKE',
    4 : 'JUICE',
}
OBJECTS_ID= {
    'VEGETABLES' : 1,
    'TEA' : 2,
    'COKE' : 3,
    'JUICE' : 4,
}

class manipuationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        # Mechanisms
        rospy.loginfo("Waiting for MoveGroupCommander ARM_TORSO...")
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso", wait_for_servers = 0)
        rospy.loginfo("Waiting for MoveGroupCommander NECK...")
        self.neck_group = moveit_commander.MoveGroupCommander("neck", wait_for_servers = 0)

        # Vision
        self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)
        rospy.loginfo("Waiting for ComputerVision 3D AS...")
        self.vision3D_as = actionlib.SimpleActionClient("Detect3D", DetectObjects3DAction)
        self.vision3D_as.wait_for_server()

        # Test Getting Objects:
        rospy.loginfo("Getting objects")
        self.get_objects()
        rospy.loginfo("Objects Received: " + str(len(self.objects)))

        # Manipulation
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = actionlib.SimpleActionClient('/pickup_pose', PickUpPoseAction)
        self.pick_as.wait_for_server()
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = actionlib.SimpleActionClient('/place_pose', PickUpPoseAction)
        self.place_as.wait_for_server()
        self.pick_goal_publisher = rospy.Publisher("pose_pickup/goal", PoseStamped, queue_size=5)
        self.place_goal_publisher = rospy.Publisher("pose_place/goal", PoseStamped, queue_size=5)
        rospy.loginfo("Loaded everything...")

        self.pick_random_object()

        # Initialize Manipulation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.manipulationServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def pick_target_object(self, object_name='Coca-Cola'):
        targetID = OBJECTS_ID[object_name]
        targetDetails = None
        for _object in self.objects:
            if  _object[0] == targetID:
                targetDetails = _object
                break
        if targetDetails == None:
            print("Object Not Detected")
            return -1

        return self.pick(targetDetails[2], targetDetails[1], [])

    def execute_cb(self, goal):
        target = goal.target_location
        rospy.loginfo("Robot Moving Towards " + target)
        # self.send_goal(placesPoses[target])
        self._as.set_succeeded(manipulationServResult(result=True))

    def send_goal(self, target_pose):
        goal = MoveBaseGoal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = target_pose
        goal.target_pose = pose_stamped
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

    def pick(self, obj_pose, obj_name, allow_contact_with_ = []):
        class PickScope:
            error_code = 0
            allow_contact_with = allow_contact_with_
            object_pose = obj_pose
            object_name = obj_name
            result_received = False
        
        def pick_feedback(feedback_msg):
            pass
        
        def pick_callback(state, result):
            PickScope.error_code = result.error_code
            PickScope.result_received = True
            rospy.loginfo("Pick Received")
            rospy.loginfo(PickScope.error_code)

        rospy.loginfo("Pick Action")
        self.pick_as.send_goal(PickUpPoseGoal(object_pose = PickScope.object_pose, object_name = PickScope.object_name, allow_contact_with = PickScope.allow_contact_with),
                    feedback_cb=pick_feedback,
                    done_cb=pick_callback)
        
        
        while not PickScope.result_received:
            pass

        return PickScope.error_code
    
    def place(self, obj_pose, obj_name, allow_contact_with_ = []):
        class PlaceScope:
            error_code = 0
            allow_contact_with = allow_contact_with_
            object_pose = obj_pose
            object_name = obj_name
            result_received = False
        
        def place_feedback(feedback_msg):
            pass
        
        def place_callback(state, result):
            PlaceScope.error_code = result.error_code
            PlaceScope.result_received = True
            rospy.loginfo("Place Received")
            rospy.loginfo(PlaceScope.error_code)

        rospy.loginfo("Place Action")
        self.place_as.send_goal(PickUpPoseGoal(object_pose = PlaceScope.object_pose, object_name = PlaceScope.object_name, allow_contact_with = PlaceScope.allow_contact_with),
                    feedback_cb=place_feedback,
                    done_cb=place_callback)
        
        while not PlaceScope.result_received:
            pass

        return PlaceScope.error_code

if __name__ == '__main__':
    rospy.init_node('manipulationServer')
    server = manipuationServer(rospy.get_name())
    rospy.spin()