#! /usr/bin/env python3

import json
import math
import tf

import numpy
import pathlib
import actionlib
import rospy
import moveit_commander
from std_msgs.msg import Bool
from object_detector.msg import DetectObjects3DAction, DetectObjects3DGoal, objectDetectionArray, objectDetection
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist 
from pick_and_place.msg import manipulationServAction, manipulationServGoal, manipulationServResult
from enum import Enum
from gpd_ros.srv import detect_grasps_samples
from gpd_ros.msg import GraspConfigList
import time

class MoveItErrorCodes(Enum):
    SUCCESS = 1
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7


OBJECTS_NAME= {
    1 : 'Coca-Cola',
    2 : 'Coffee',
    3 : 'Nesquik',
}
OBJECTS_ID= {
    'Coca-Cola' : 1,
    'Coffee' : 2,
    'Nesquik' : 3,
}
class ManipulationGoals(Enum):
    COKE = 1
    COFFEE = 2
    NESQUIK = 3
    BIGGEST = 4

class manipuationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        self.ARM_GROUP = rospy.get_param("ARM_GROUP", "arm_torso")
        self.HEAD_GROUP = rospy.get_param("HEAD_GROUP", "head")

        # # Mechanisms
        rospy.loginfo("Waiting for MoveGroupCommander ARM_TORSO...")
        self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP, wait_for_servers = 0)
        rospy.loginfo("Waiting for MoveGroupCommander HEAD/HEAD...")
        self.head_group = moveit_commander.MoveGroupCommander(self.HEAD_GROUP, wait_for_servers = 0)

        # Initialize Robot Pose
        self.initARM()
        self.initHEAD()
        
        # Vision
        self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)
        rospy.loginfo("Waiting for ComputerVision 3D AS...")
        self.vision3D_as = actionlib.SimpleActionClient("Detect3D", DetectObjects3DAction)
        self.vision3D_as.wait_for_server()

        # # Manipulation
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = actionlib.SimpleActionClient('/pickup_pose', PickAndPlaceAction)
        self.pick_as.wait_for_server()
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = actionlib.SimpleActionClient('/place_pose', PickAndPlaceAction)
        self.place_as.wait_for_server()
        self.pick_goal_publisher = rospy.Publisher("pose_pickup/goal", PoseStamped, queue_size=5)
        self.place_goal_publisher = rospy.Publisher("pose_place/goal", PoseStamped, queue_size=5)
        self.grasp_config_list = rospy.Publisher("grasp_config_list", GraspConfigList, queue_size=5)

        rospy.wait_for_service('/detect_grasps_server_samples/detect_grasps_samples')
        rospy.loginfo("Loaded everything...")
        # self.pick_random_object()

        # Initialize Manipulation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, manipulationServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def initARM(self):
        ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
        ARM_GRASP = rospy.get_param("ARM_GRASP", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_state = JointState()
        joint_state.name = ARM_JOINTS
        joint_state.position = ARM_GRASP
        self.arm_group.go(joint_state, wait=True)
        self.arm_group.stop()
    
    def initHEAD(self):
        HEAD_JOINTS = rospy.get_param("HEAD_JOINTS", ["head_1_joint", "head_2_joint"])
        HEAD_INIT = rospy.get_param("HEAD_LOOK_DOWN", [0.0, 0.0])
        joint_state = JointState()
        joint_state.name = HEAD_JOINTS
        joint_state.position = HEAD_INIT
        self.head_group.go(joint_state, wait=True)
        self.head_group.stop()
    
    def headTableDiscovery(self):
        HEAD_JOINTS = rospy.get_param("HEAD_JOINTS", ["head_1_joint", "head_2_joint"])
        HEAD_INIT = rospy.get_param("HEAD_LEFT", [0.0, 0.0])
        joint_state = JointState()
        joint_state.name = HEAD_JOINTS
        joint_state.position = HEAD_INIT
        self.head_group.go(joint_state, wait=True)
        self.head_group.stop()
        HEAD_JOINTS = rospy.get_param("HEAD_JOINTS", ["head_1_joint", "head_2_joint"])
        HEAD_INIT = rospy.get_param("HEAD_RIGHT", [0.0, 0.0])
        joint_state = JointState()
        joint_state.name = HEAD_JOINTS
        joint_state.position = HEAD_INIT
        self.head_group.go(joint_state, wait=True)
        self.head_group.stop()
        self.initHEAD()
        
        

    def execute_cb(self, goal):
        target = ManipulationGoals(goal.object_id)
        self.headTableDiscovery()

        # Get Objects:
        rospy.loginfo("Getting objects")
        found = self.get_object(target)
        if not found:
            rospy.loginfo("Object Not Found")
            self._as.set_succeeded(manipulationServResult(result = False))
            return
        rospy.loginfo("Object Found")
            
        TEST_GDP = False
        while TEST_GDP  and not rospy.is_shutdown():
            rospy.loginfo("Getting grasping points")
            grasping_points = self.get_grasping_points()
            rospy.loginfo("Grasping Points Found")
            rospy.loginfo("Waiting for user input")
            input("Press Enter to retry...")

        grasping_points = self.get_grasping_points()
        if grasping_points is None:
            rospy.loginfo("Grasping Points Not Found")
            self._as.set_succeeded(manipulationServResult(result = False))
            return

        self.grasp_config_list.publish(grasping_points)
        rospy.loginfo("Robot Picking " + target.name + " up")
        result = self.pick(self.object_pose, "current", allow_contact_with_ = ["<octomap>"], grasping_points = grasping_points)
        if result != 1:
            rospy.loginfo("Pick Failed")
            self._as.set_succeeded(manipulationServResult(result = False))
            return
        rospy.loginfo("Robot Picked " + target.name + " up")
        ## Move Up
        self.object_pose.pose.position.z += 0.2
        self.arm_group.set_pose_target(self.object_pose)
        self.arm_group.plan()
        self.arm_group.go(wait=True)
        self.arm_group.stop()    
        self._as.set_succeeded(manipulationServResult(result = True))
    
    def get_grasping_points(self):
        attempts = 0
        while attempts < 3:
            try:
                detect_grasps = rospy.ServiceProxy('/detect_grasps_server_samples/detect_grasps_samples', detect_grasps_samples)
                detect_grasps.wait_for_service()
                attempts += 1
                rospy.loginfo("Getting Grasping Points")
                resp = detect_grasps(self.object_cloud).grasp_configs
                if len(resp.grasps) > 0:
                    return resp
                rospy.loginfo("No Grasping Points Found")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)
            time.sleep(0.5)
        return None
        

    def pick(self, obj_pose, obj_name, allow_contact_with_ = [], grasping_points = []):
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
        goal = PickAndPlaceGoal(grasp_config_list = grasping_points, target_pose = PickScope.object_pose, object_name = PickScope.object_name, allow_contact_with = PickScope.allow_contact_with)
        self.pick_as.send_goal(goal = goal,
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
        self.place_as.send_goal(PickAndPlaceGoal(target_pose = PlaceScope.object_pose, object_name = PlaceScope.object_name, allow_contact_with = PlaceScope.allow_contact_with),
                    feedback_cb=place_feedback,
                    done_cb=place_callback)
        
        while not PlaceScope.result_received:
            pass

        return PlaceScope.error_code

    def get_object(self, target = ManipulationGoals['BIGGEST']):
        
        class GetObjectsScope:
            success = False
            detection = objectDetection()
            object_pose = []
            object_cloud = []
            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            result_received = False
        
        def get_objects_feedback(feedback_msg):
            GetObjectsScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            GetObjectsScope.success = result.success
            GetObjectsScope.object_pose = result.object_pose
            GetObjectsScope.object_cloud = result.object_cloud
            GetObjectsScope.x_plane = result.x_plane
            GetObjectsScope.y_plane = result.y_plane
            GetObjectsScope.z_plane = result.z_plane
            GetObjectsScope.width_plane = result.width_plane
            GetObjectsScope.height_plane = result.height_plane
            GetObjectsScope.result_received = True

        if target == ManipulationGoals['BIGGEST']:
            goal = DetectObjects3DGoal()
        else:
            # Search for Target
            attempts = 0
            success = False
            while attempts < 3:
                try:
                    detections = rospy.wait_for_message("/detections", objectDetectionArray, timeout=5.0)
                except:
                    attempts += 1
                    continue

                if len(detections.detections) == 0:
                    attempts += 1
                    continue
                rospy.loginfo("Objects Found:" + str(len(detections.detections)))
                for detection in detections.detections:
                    if detection.labelText == OBJECTS_NAME[target.value]:
                        goal = DetectObjects3DGoal(force_object = objectDetectionArray(detections = [detection]))
                        GetObjectsScope.detection = detection
                        success = True
                        break
                if success:
                    break
                attempts+=1
            if not success:
                return False

        attempts = 0
        while attempts < 3:
            self.vision3D_as.send_goal(
                goal,
                feedback_cb=get_objects_feedback,
                done_cb=get_result_callback)
        
            start_time = time.time()
            while not GetObjectsScope.result_received:
                pass
        
            if GetObjectsScope.success:
                break
            attempts += 1

        if not GetObjectsScope.success:
            return False

        self.object_pose = GetObjectsScope.object_pose
        self.object_cloud = GetObjectsScope.object_cloud

        return GetObjectsScope.success

if __name__ == '__main__':
    rospy.init_node('manipulationServer')
    server = manipuationServer(rospy.get_name())
    rospy.spin()