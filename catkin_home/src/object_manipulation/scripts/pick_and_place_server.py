#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from object_manipulation.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from gazebo_msgs.srv import GetModelState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from copy import deepcopy
from random import shuffle
import copy
from enum import Enum
import math

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
                     grasp_pose=PoseStamped(),
                     possible_grasps=[],
                     links_to_allow_contact=[],
                     support_surface_name = "table"):
    """ Create a PickupGoal with the provided data"""
    pug = PickupGoal()
    pug.target_name = target
    pug.group_name = group
    pug.possible_grasps.extend(possible_grasps)
    pug.support_surface_name = support_surface_name
    pug.allow_gripper_support_collision = True
    pug.allowed_planning_time = 30.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 3
    pug.allowed_touch_objects = ['<octomap>']
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)
    return pug

def createPlaceGoal(place_pose,
                    place_locations,
                    group="arm_torso",
                    target="part",
                    links_to_allow_contact=[],
                    support_surface_name = "<octomap>"):
    """Create PlaceGoal with the provided data"""
    placeg = PlaceGoal()
    placeg.group_name = group
    placeg.support_surface_name = support_surface_name
    placeg.allow_gripper_support_collision = True
    placeg.attached_object_name = target
    placeg.place_locations = place_locations
    placeg.allowed_planning_time = 30.0
    placeg.planning_options.planning_scene_diff.is_diff = True
    placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
    placeg.planning_options.plan_only = False
    placeg.planning_options.replan = True
    placeg.planning_options.replan_attempts = 3
    placeg.allowed_touch_objects = ['<octomap>']
    placeg.allowed_touch_objects.extend(links_to_allow_contact)

    return placeg

class Can:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class PickAndPlaceServer(object):
    def __init__(self):
        self._blockListDict = {
            'can_1': Can('standard_can_fit_clone', 'link_0'),
            'can_2': Can('standard_can_fit_clone_0', 'link_0'),
            'can_3': Can('standard_can_fit_clone_1', 'link_0'),
            'can_4': Can('standard_can_fit_clone_2', 'link_0'),
            'can_5': Can('standard_can_fit_clone_4_clone', 'link_0'),
            'can_6': Can('standard_can_fit_clone_4_clone_0', 'link_0'),
            'can_7': Can('standard_can_fit_clone_4_clone_1', 'link_0'),
            'can_8': Can('standard_can_fit_clone_4_clone_2', 'link_0'),
            'can_9': Can('standard_can_fit_clone_4_clone_5', 'link_0'),
            'can_10': Can('standard_can_fit_clone_4_clone_6', 'link_0'),
            'can_11': Can('standard_can_fit_clone_4_clone_7', 'link_0'),
            'can_12': Can('standard_can_fit_clone_4_clone_8', 'link_0'),
        }
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

        self.is_simulation = rospy.get_param('/IS_SIMULATION', True)
        rospy.logwarn("IS_SIMULATION " + str(self.is_simulation))

        rospy.loginfo("Waiting for Robot...")
        odom_msg = rospy.wait_for_message("/mobile_base_controller/odom", Odometry)
        rospy.loginfo("Robot Launched...")

        rospy.loginfo("Initalizing PickAndPlaceServer...")
        self.sg = SphericalGrasps()
        rospy.loginfo("Connecting to pickup AS")
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        rospy.loginfo("Connecting to place AS")
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")
        self.scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")

        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo("Connected!")

        if self.is_simulation:
            rospy.loginfo("Waiting for /gazebo/get_model_state")
            rospy.wait_for_service("/gazebo/get_model_state")
            self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.pose_array_p = rospy.Publisher("pose_array/cans", PoseArray, queue_size=5)
            rospy.loginfo("Waiting for /link_attacher_node/attach Service")
            rospy.wait_for_service("/link_attacher_node/attach")
            self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
            rospy.loginfo("Waiting for /link_attacher_node/detach Service")
            rospy.wait_for_service("/link_attacher_node/detach")
            self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers = 0)

        # Get the links of the end effector exclude from collisions
        self.links_to_allow_contact_right = rospy.get_param('~links_to_allow_contact_right', None)
        self.links_to_allow_contact_left = rospy.get_param('~links_to_allow_contact_left', None)
        if self.links_to_allow_contact_right is None or self.links_to_allow_contact_left is None:
            rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact_right | ~links_to_allow_contact_left")

        self.pick_as = SimpleActionServer(
            '/pickup_pose', PickUpPoseAction,
            execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

        self.place_as = SimpleActionServer(
            '/place_pose', PickUpPoseAction,
            execute_cb=self.place_cb, auto_start=False)
        self.place_as.start()

        self.can_poses_gazebo = []
        self.currentAttach = None
        loadCansThread = threading.Thread(target=self.load_cans_gazebo, args=(True, ))
        loadCansThread.start()

    def load_cans_gazebo(self, wait = False):
        if not self.is_simulation:
            return
        if wait:
            time.sleep(7.5)
        rospy.logwarn("Loading Gazebo Requirements For Attacher...")
        self.can_poses_gazebo = []

        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
    
        for block in self._blockListDict.itervalues():
            blockName = str(block._name)
            resp_coordinates = self.get_model_srv(blockName, "map")
            resp_coordinates.pose.position.x = resp_coordinates.pose.position.x + 2.01
            resp_coordinates.pose.position.y = resp_coordinates.pose.position.y + ARGS["CAN_RADIUS"]
            pa.poses.append(resp_coordinates.pose)
            self.can_poses_gazebo.append([resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z, block])

        self.pose_array_p.publish(pa)

    def generate_attach_arm_to_can_msg(self, can_name = None):
        if not self.is_simulation:
            return

        if can_name is None or len(self.can_poses_gazebo) == 0:
            return

        scene_objects = self.scene.get_objects(object_ids = [can_name])
        if not (can_name in scene_objects):
            return
        
        if len(scene_objects[can_name].primitive_poses) == 0:
            return

        pose_stamped = PoseStamped()
        pose_stamped.header = scene_objects[can_name].header
        pose_stamped.pose = scene_objects[can_name].primitive_poses[0]
        pose_stamped = self.transformTo(pose_stamped, frame_id="map")        

        def get_distance_to_ref(x_, y_, z_):
            return math.sqrt( (x_ *x_) + (y_* y_)+(z_*z_))
        
        # Match Model with name by distance to current Pose.
        self.can_poses_gazebo.sort(key=lambda elem: get_distance_to_ref(
                                elem[0]-pose_stamped.pose.position.x,
                                elem[1]-pose_stamped.pose.position.y,
                                0.0))

        req = AttachRequest()
        req.model_name_1 = "tiago_dual" # Robot
        req.link_name_1 = "gripper_right_finger_link" # Gripper Link
        req.model_name_2 = str(self.can_poses_gazebo[0][3]._name) # Model can
        req.link_name_2 = str(self.can_poses_gazebo[0][3]._relative_entity_name)  # Link can

        self.currentAttach = req

    def detach_can(self):
        if not self.is_simulation:
            return

        if self.currentAttach == None:
            return
        self.detach_srv.call(self.currentAttach)
        rospy.logwarn("Detached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))
        self.currentAttach = None
        self.load_cans_gazebo()
    
    def attach_can(self):
        if not self.is_simulation:
            return

        if self.currentAttach == None:
            return
        self.attach_srv.call(self.currentAttach)
        rospy.logwarn("Attached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))

    def detach_and_retach(self, waitTime = 0):
        if not self.is_simulation:
            return

        if self.currentAttach == None:
            return
        time.sleep(waitTime)
        self.detach_srv.call(self.currentAttach)
        rospy.logwarn("Detached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))
        time.sleep(1) ## Give some time to acommodate between grippers
        self.attach_srv.call(self.currentAttach)
        rospy.logwarn("Attached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))

    def pick_cb(self, goal):
        """
        :type goal: PickUpPoseGoal
        """
        error_code = self.grasp_object(goal.object_pose, goal.object_name, goal.allow_contact_with)
        p_res = PickUpPoseResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    def place_cb(self, goal):
        """
        :type goal: PickUpPoseGoal
        """
        error_code = self.place_object(goal.object_pose, goal.object_name, goal.allow_contact_with)
        p_res = PickUpPoseResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.place_as.set_aborted(p_res)
        else:
            self.place_as.set_succeeded(p_res)

    def confirm_status_with_attached_objects(self, code_error, object_name, expected):
        rospy.sleep(0.5) # Give some time to appear in attached_objects
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        if expected:
            if code_error == 1 and not (object_name in scene_attached_objects):
                code_error = -1
            
            if code_error != 1 and (object_name in scene_attached_objects):
                code_error = 1
        else:
            if code_error == 1 and (object_name in scene_attached_objects):
                code_error = -1
            
            if code_error != 1 and not (object_name in scene_attached_objects):
                code_error = 1

        return code_error

    def grasp_object(self, object_pose, object_name, allow_contact_with = []):
        # Confirm if not attached already
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        rospy.loginfo("Attached objects:" + str(scene_attached_objects))
        if object_name in scene_attached_objects:
            return

        self.generate_attach_arm_to_can_msg(object_name)

        self.scene.remove_attached_object("arm_right_tool_link")

        possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)

        links_to_allow_contact = self.links_to_allow_contact_right

        links_to_allow_contact.extend(allow_contact_with)

        goal = createPickupGoal(
            "arm_torso", object_name, object_pose, possible_grasps, links_to_allow_contact)
        
        error_code = self.handle_pick_as(goal)

        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, True)

        rospy.logwarn(
            "Pick result: " +
        str(moveit_error_dict[error_code]))

        return error_code

    def handle_pick_as(self, goal):
        class PickScope:
            state = ""
            error_code = 0
            result_received = False

        def pick_feedback(feedback_msg):
            PickScope.state = feedback_msg.state
            pass
        
        def pick_callback(state, result):
            rospy.logwarn("Result Received")
            PickScope.error_code = result.error_code.val
            PickScope.result_received = True

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal,
                    feedback_cb=pick_feedback,
                    done_cb=pick_callback)
        
        gripperMovement = False
        while not PickScope.result_received:
            g_joint_values = self.gripper_group.get_current_joint_values()
            if not gripperMovement and g_joint_values[0] < 0.040 and g_joint_values[1] < 0.040:
                rospy.logwarn("Gripper Movement, Attaching object")
                self.attach_can()
                gripperMovement = True

        return PickScope.error_code

    def handle_place_as(self, goal):
        class PlaceScope:
            state = ""
            error_code = 0
            result_received = False

        def place_feedback(feedback_msg):
            PlaceScope.state = feedback_msg.state
            rospy.logwarn("Feedback Received: " + str(PlaceScope.state))
            pass
        
        def place_callback(state, result):
            rospy.logwarn("Result Received")
            PlaceScope.error_code = result.error_code.val
            PlaceScope.result_received = True

        rospy.loginfo("Sending goal")
        self.place_ac.send_goal(goal,
                    feedback_cb=place_feedback,
                    done_cb=place_callback)
        
        gripperMovement = False
        while not PlaceScope.result_received:
            g_joint_values = self.gripper_group.get_current_joint_values()
            if not gripperMovement and g_joint_values[0] > 0.040 and g_joint_values[1] > 0.040:
                rospy.logwarn("Gripper Movement, Detaching object")
                self.detach_can()
                gripperMovement = True

        return PlaceScope.error_code
    
    def place_object(self, object_pose, object_name, allow_contact_with = [], try_only_with_arm_first = False):

        possible_placings = self.sg.create_placings_from_object_pose(object_pose)
        
        links_to_allow_contact = self.links_to_allow_contact_left

        links_to_allow_contact.extend(allow_contact_with)

        error_code = -1

        
        rospy.loginfo(
            "Trying to place with arm and torso")
        # Try with arm and torso
        goal = createPlaceGoal(
            object_pose, possible_placings, "arm_torso", object_name, links_to_allow_contact)

        error_code = self.handle_place_as(goal)
    
        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, False)

        rospy.logwarn(
            "Place Result: " +
        str(moveit_error_dict[error_code]))

        return error_code

    def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s

    def transformTo(self, poseStamped, frame_id="base_footprint"):
        poseStamped.header.frame_id = self.strip_leading_slash(poseStamped.header.frame_id)
        rospy.loginfo("Transforming Pose from frame: " + poseStamped.header.frame_id + " to "+ frame_id)
        ps = PoseStamped()
        ps.pose = poseStamped.pose
        ps.header.stamp = self.tfBuffer.get_latest_common_time(frame_id, poseStamped.header.frame_id)
        ps.header.frame_id = poseStamped.header.frame_id
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform(frame_id, 
                                       ps.header.frame_id,
                                       rospy.Time(0))
                poseStampedTF = do_transform_pose(ps, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
                ps.header.stamp = self.tfBuffer.get_latest_common_time(frame_id, poseStamped.header.frame_id)
        return poseStampedTF

if __name__ == '__main__':
    rospy.init_node('pick_and_place_server')
    paps = PickAndPlaceServer()
    rospy.spin()
