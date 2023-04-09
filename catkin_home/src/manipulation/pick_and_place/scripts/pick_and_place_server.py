#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
import tf.transformations as transformations
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation, AllowedCollisionEntry, PlanningSceneComponents
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal, PickAndPlaceResult, PickAndPlaceFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Header
from gpd_ros.msg import GraspConfig, GraspConfigList
import numpy
import math
import copy

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class PickAndPlaceServer(object):
    ARM_GROUP = "whole_body"
    GRIPPER_GROUP="gripper"
    GRASP_POSTURES_FRAME_ID="arm_tool_link"
    GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    GRIPPER_OPEN = [0.044, 0.044]
    GRIPPER_CLOSED = [0.009, 0.009]
    ALLOW_CONTACT = ["arm_tool_link", "<octomap>", "base_link"]

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        
        # Load parameters
        PickAndPlaceServer.ARM_GROUP = rospy.get_param("ARM_GROUP", PickAndPlaceServer.ARM_GROUP)
        PickAndPlaceServer.GRIPPER_GROUP = rospy.get_param("GRIPPER_GROUP", PickAndPlaceServer.GRIPPER_GROUP)
        PickAndPlaceServer.GRASP_POSTURES_FRAME_ID = rospy.get_param("GRASP_POSTURES_FRAME_ID", PickAndPlaceServer.GRASP_POSTURES_FRAME_ID)
        PickAndPlaceServer.ALLOW_CONTACT = rospy.get_param("ALLOW_CONTACT", PickAndPlaceServer.ALLOW_CONTACT)
        PickAndPlaceServer.GRIPPER_JOINTS = rospy.get_param("GRIPPER_JOINTS", PickAndPlaceServer.GRIPPER_JOINTS)
        PickAndPlaceServer.GRIPPER_OPEN = rospy.get_param("GRIPPER_OPEN", PickAndPlaceServer.GRIPPER_OPEN)
        PickAndPlaceServer.GRIPPER_CLOSED = rospy.get_param("GRIPPER_CLOSED", PickAndPlaceServer.GRIPPER_CLOSED)

        rospy.loginfo("Initalizing PickAndPlaceServer...")
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
        self.scene_srv.wait_for_service(timeout=100)
        rospy.loginfo("Connected.")
        print("Current allowed collisions:")
        self.curr_collision_matrix = self.scene_srv(PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)).scene.allowed_collision_matrix
        self.curr_collision_matrix.entry_names.append("<octomap>")
        for entry in self.curr_collision_matrix.entry_values:
            entry.enabled.append(True)
        self.curr_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True for i in range(len(self.curr_collision_matrix.entry_names))]))
        
        rospy.loginfo("Connecting to clear octomap service...")
        self.clear_octomap_srv = rospy.ServiceProxy(
            '/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service(timeout=100)
        rospy.loginfo("Connected!")
        
        self.gripper_group = moveit_commander.MoveGroupCommander(PickAndPlaceServer.GRIPPER_GROUP, wait_for_servers = 0)

        rospy.loginfo("Gripper Group Up!")
        self.pick_as = SimpleActionServer(
            '/pickup_pose', PickAndPlaceAction,
            execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

        rospy.loginfo("Action Server1 Up!")
        self.place_as = SimpleActionServer(
            '/place_pose', PickAndPlaceAction,
            execute_cb=self.place_cb, auto_start=False)
        self.place_as.start()
        
        self.pose_array_p = rospy.Publisher("pose_array/object", PoseArray, queue_size=10)

        rospy.loginfo("Waiting for MoveGroupCommander...")
        self.arm_group = moveit_commander.MoveGroupCommander(PickAndPlaceServer.ARM_GROUP, wait_for_servers = 0)
        
        rospy.loginfo("Action Server Pick & Place Up!")

    
    # Function to handle pickup callback.
    def pick_cb(self, goal):
        """
        :type goal: PickAndPlaceGoal
        """
        links_to_allow_contact = PickAndPlaceServer.ALLOW_CONTACT
        links_to_allow_contact.extend(goal.allow_contact_with)
        grasps = gpd_to_moveit_new(goal.grasp_config_list, links_to_allow_contact)
        
        pa = PoseArray()
        pS = PoseStamped()
        TEST_ORIENTATION = False
        for grasp in grasps:
            pS = grasp.grasp_pose
            
            if TEST_ORIENTATION:
                pS.pose.position.z += 0.2 # Avoid Collision with table increasing Z.
            
            pa.header = pS.header
            pa.poses.append(pS.pose)

            if TEST_ORIENTATION:
                break # Only test one pose.

        if TEST_ORIENTATION:
            rospy.loginfo("Confirming Orientation")
            self.arm_group.set_pose_target(pS)
            self.arm_group.plan()
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.pose_array_p.publish(pa)
            self.pick_as.set_succeeded(p_res)
            return
        
        self.pose_array_p.publish(pa)

        error_code = self.grasp_object(grasps, goal.object_name, links_to_allow_contact)
        p_res = PickAndPlaceResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    # Function to handle place callback.
    def place_cb(self, goal):
        """
        :type goal: PickAndPlaceGoal
        """
        error_code = self.place_object(goal.target_pose, goal.object_name, goal.allow_contact_with)
        p_res = PickAndPlaceResult()
        p_res.error_code = error_code
        if error_code != 1:
            self.place_as.set_aborted(p_res)
        else:
            self.place_as.set_succeeded(p_res)

    # Function to confirm that an object is or not attached.
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

    # Function to grasp an object.
    def grasp_object(self, grasps, object_name, allow_contact_with = []):
        # Confirm if not attached already
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        rospy.loginfo("Attached objects:" + str(scene_attached_objects))
        if object_name in scene_attached_objects:
            return

        possible_grasps = grasps

        goal = createPickupGoal(
            PickAndPlaceServer.ARM_GROUP, object_name, possible_grasps, allow_contact_with, "<octomap>" , self.curr_collision_matrix)
        
        error_code = self.handle_pick_as(goal)

        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, True)

        rospy.logwarn(
            "Pick result: " +
        str(moveit_error_dict[error_code]))

        return error_code

    # Function to handle pick action server call.
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
        
        while not PickScope.result_received:
            pass

        return PickScope.error_code

    # Function to handle place action server call.
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
        
        while not PlaceScope.result_received:
            pass

        return PlaceScope.error_code
    
    # Function to place an object.
    def place_object(self, object_pose, object_name, allow_contact_with = []):

        links_to_allow_contact = PickAndPlaceServer.ALLOW_CONTACT
        links_to_allow_contact.extend(allow_contact_with)
        
        possible_placings = create_placings_from_object_pose(object_pose, links_to_allow_contact)

        error_code = -1
        
        rospy.loginfo("Trying to place with arm and torso")
        
        goal = createPlaceGoal(
            possible_placings, PickAndPlaceServer.ARM_GROUP, object_name, links_to_allow_contact, "<octomap>", self.curr_collision_matrix)

        error_code = self.handle_place_as(goal)
    
        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, False)

        rospy.logwarn(
            "Place Result: " +
        str(moveit_error_dict[error_code]))

        return error_code


# Function to create a PickupGoal with the provided data.
def createPickupGoal(group=PickAndPlaceServer.ARM_GROUP, target="part",
                     possible_grasps=[],
                     links_to_allow_contact=[],
                     support_surface_name = "<octomap>",
                     curr_collision_matrix = None):
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
    pug.allowed_touch_objects.extend(links_to_allow_contact)
    pug.attached_object_touch_links = ['<octomap>']
    pug.attached_object_touch_links.extend(links_to_allow_contact)
    pug.planning_options.planning_scene_diff.allowed_collision_matrix = curr_collision_matrix
    return pug

# Function to create a PlaceGoal with the provided data..
def createPlaceGoal(place_locations,
                    group=PickAndPlaceServer.ARM_GROUP,
                    target="part",
                    links_to_allow_contact=[],
                    support_surface_name = "<octomap>",
                    curr_collision_matrix = None):
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
    placeg.allowed_touch_objects = links_to_allow_contact
    placeg.planning_options.planning_scene_diff.allowed_collision_matrix = curr_collision_matrix
    return placeg

# Function to create a list of possible placings from a PoseStamped.
def create_placings_from_object_pose(posestamped, allowed_touch_objects):
    """ Set PlaceLocation according to Pose sended """
    grasp_postures_frame_id = PickAndPlaceServer.GRASP_POSTURES_FRAME_ID
    gripper_joint_names = PickAndPlaceServer.GRIPPER_JOINTS
    time_pre_grasp_posture = 0.8
    gripper_grasp_positions = PickAndPlaceServer.GRIPPER_CLOSED
    gripper_pre_grasp_positions = PickAndPlaceServer.GRIPPER_OPEN
    

    place_locs = []
    pre_grasp_posture = JointTrajectory()
    # Actually ignored....
    pre_grasp_posture.header.frame_id = grasp_postures_frame_id
    pre_grasp_posture.joint_names = gripper_joint_names
    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = gripper_pre_grasp_positions
    jtpoint.time_from_start = rospy.Duration(time_pre_grasp_posture)
    pre_grasp_posture.points.append(jtpoint)
    
    pl = PlaceLocation()
    pl.place_pose = posestamped
    pl.pre_place_approach = createGripperTranslation(
        Vector3(1.0, 0.0, 0.0), desired_distance = 0.20, min_distance = 0.0)
    pl.post_place_retreat = createGripperTranslation(
        Vector3(-1.0, -0.30, 0.0), desired_distance = 0.20, min_distance = 0.0)

    pl.post_place_posture = pre_grasp_posture
    pl.allowed_touch_objects = allowed_touch_objects
    place_locs.append(pl)

    return place_locs

# Function to create a list of possible grasps from a PoseStamped.
def create_grasp(poseStamped, allowed_touch_objects, grasp_config):
    """
    :type pose: Pose
        pose of the gripper for the grasp
    :rtype: Grasp
    """
    grasp_postures_frame_id = PickAndPlaceServer.GRASP_POSTURES_FRAME_ID
    gripper_joint_names = PickAndPlaceServer.GRIPPER_JOINTS
    time_pre_grasp_posture = 0.8
    time_grasp_posture = 0.2
    time_grasp_posture_final = 0.5
    gripper_grasp_positions = PickAndPlaceServer.GRIPPER_CLOSED

    # mapToAngel = [(0.15, 1.239), (0.102, 0.75), (0.09, 0.506145), (0.05, 0.314159)]
    # width_angle = 1.239
    # for elem in mapToAngel:
    #     if grasp_config.width.data < elem[0]:
    #         width_angle = elem[1]

    gripper_pre_grasp_positions = PickAndPlaceServer.GRIPPER_OPEN
    fix_tool_frame_to_grasping_frame_roll = -90.0
    fix_tool_frame_to_grasping_frame_pitch = 0
    fix_tool_frame_to_grasping_frame_yaw = 0.0
    grasp_desired_distance = 0.20
    grasp_min_distance = 0.0
    max_contact_force = 0.0
    g = Grasp()
    
    pre_grasp_posture = JointTrajectory()
    pre_grasp_posture.header.frame_id = grasp_postures_frame_id
    pre_grasp_posture.joint_names = gripper_joint_names

    jtpoint = JointTrajectoryPoint()
    jtpoint.positions = gripper_pre_grasp_positions
    jtpoint.time_from_start = rospy.Duration(time_pre_grasp_posture)
    pre_grasp_posture.points.append(jtpoint)

    grasp_posture = copy.deepcopy(pre_grasp_posture)
    grasp_posture.points[0].time_from_start = rospy.Duration(
        time_pre_grasp_posture + time_grasp_posture)
    jtpoint2 = JointTrajectoryPoint()
    jtpoint2.positions = gripper_grasp_positions
    jtpoint2.time_from_start = rospy.Duration(
        time_pre_grasp_posture +
        time_grasp_posture + time_grasp_posture_final)
    grasp_posture.points.append(jtpoint2)

    g.pre_grasp_posture = pre_grasp_posture
    g.grasp_posture = grasp_posture

    header = poseStamped.header
    q = [poseStamped.pose.orientation.x, poseStamped.pose.orientation.y,
            poseStamped.pose.orientation.z, poseStamped.pose.orientation.w]
    # Fix orientation from gripper_link to parent_link (tool_link)
    fix_tool_to_gripper_rotation_q = transformations.quaternion_from_euler(
        math.radians(fix_tool_frame_to_grasping_frame_roll),
        math.radians(fix_tool_frame_to_grasping_frame_pitch),
        math.radians(fix_tool_frame_to_grasping_frame_yaw)
    )
    q = transformations.quaternion_multiply(q, fix_tool_to_gripper_rotation_q)
    fixed_pose = copy.deepcopy(poseStamped.pose)
    fixed_pose.orientation = Quaternion(*q)

    g.grasp_pose = PoseStamped(header, fixed_pose)
    g.grasp_quality = grasp_config.score.data # min(1000, max(0, grasp_quality)) / 1000

    g.pre_grasp_approach = createGripperTranslation(
        Vector3(1.0, 0.0, 0.0), desired_distance = grasp_desired_distance,
        min_distance = grasp_min_distance)
    
    g.post_grasp_retreat = createGripperTranslation(
        Vector3(-1.0, -0.35, 0.0), desired_distance = grasp_desired_distance,
        min_distance = grasp_min_distance)

    g.max_contact_force = max_contact_force
    g.allowed_touch_objects = allowed_touch_objects

    return g

# Function that creates a GripperTranslation message.
def createGripperTranslation(direction_vector, desired_distance, min_distance):
    """Returns a GripperTranslation message with the
        direction_vector and desired_distance and min_distance in it.
    Intended to be used to fill the pre_grasp_approach
        and post_grasp_retreat field in the Grasp message."""
    g_trans = GripperTranslation()

    g_trans.direction.header.frame_id = PickAndPlaceServer.GRASP_POSTURES_FRAME_ID

    g_trans.direction.header.stamp = rospy.Time.now()
    g_trans.direction.vector.x = direction_vector.x
    g_trans.direction.vector.y = direction_vector.y
    g_trans.direction.vector.z = direction_vector.z
    g_trans.desired_distance = desired_distance
    g_trans.min_distance = min_distance
    return g_trans

# Function that converts from GPD to MoveIt! Grasp message.
def gpd_to_moveit_new(grasp_config_list, allow_contact_with):
    header = Header()
    header.frame_id = "map"
    kThresholdScore = 1.0
    res = []
    for grasp_config in grasp_config_list.grasps:
        if grasp_config.score.data < kThresholdScore:
                continue
        rospy.loginfo("grasp score is %f", grasp_config.score.data)
        rospy.loginfo("grasp required width %f", grasp_config.width.data)

        # Set grasp position, translation from hand-base to the parent-link of EEF
        grasp_pose = PoseStamped()
        grasp_pose.header = header
        
        grasp_pose.pose.position.x = grasp_config.position.x
        grasp_pose.pose.position.y = grasp_config.position.y
        grasp_pose.pose.position.z = grasp_config.position.z # +0.15

        # Rotation Matrix
        rot = numpy.array([[grasp_config.approach.x, grasp_config.binormal.x, grasp_config.axis.x],
                        [grasp_config.approach.y, grasp_config.binormal.y, grasp_config.axis.y],
                        [grasp_config.approach.z, grasp_config.binormal.z, grasp_config.axis.z]])
        R = numpy.eye(4)
        R[:3, :3] = rot
        quat = transformations.quaternion_from_matrix(R)

        # EEF yaw-offset to its parent-link (last link of arm)
        eef_yaw_offset = 0.0
        offquat = transformations.quaternion_about_axis(eef_yaw_offset, (0, 0, 1))
        quat = transformations.quaternion_multiply(quat, offquat)
        quat = transformations.unit_vector(quat)
        # Set grasp orientation
        grasp_pose.pose.orientation.x = quat[0]
        grasp_pose.pose.orientation.y = quat[1]
        grasp_pose.pose.orientation.z = quat[2]
        grasp_pose.pose.orientation.w = quat[3]

        res.append(create_grasp(grasp_pose, allow_contact_with, grasp_config))
    return res

# Function that converts from GPD to MoveIt! Grasp message.
def gpd_to_moveit(grasp_config_list):
    header = Header()
    header.frame_id = "map"
    approach_distance = 0.1
    eef_yaw_offset = 0.0
    eef_offset = 0.154 # M_PI / 4;
    table_height = 0.489854
    object_height_min = 0.028
    kThresholdScore = 1.0
    res = []
    for grasp_config in grasp_config_list.grasps:
        msg = Grasp()
        msg.grasp_pose.header = header
        if grasp_config.score.data < kThresholdScore:
            continue
        # TODO: Use kThresholdScore according to this.
        msg.grasp_quality = grasp_config.score.data
        rospy.loginfo("grasp score is %f", grasp_config.score.data)

        offset = eef_offset
        # Make sure a distance of 'object_height_min/2' from tabletop to fingertip.
        # offset += (table_height + object_height_min / 2) - grasp_config.position.z
        rospy.loginfo("offset is %f", offset)

        # Set grasp position, translation from hand-base to the parent-link of EEF
        msg.grasp_pose.pose.position.x = grasp_config.position.x - grasp_config.approach.x * offset
        msg.grasp_pose.pose.position.y = grasp_config.position.y - grasp_config.approach.y * offset
        msg.grasp_pose.pose.position.z = grasp_config.position.z - grasp_config.approach.z * offset

        # Rotation Matrix
        rot = numpy.array([[grasp_config.approach.x, grasp_config.binormal.x, grasp_config.axis.x],
                        [grasp_config.approach.y, grasp_config.binormal.y, grasp_config.axis.y],
                        [grasp_config.approach.z, grasp_config.binormal.z, grasp_config.axis.z]])
        R = numpy.eye(4)
        R[:3, :3] = rot
        quat = transformations.quaternion_from_matrix(R)

        # EEF yaw-offset to its parent-link (last link of arm)
        offquat = transformations.quaternion_about_axis(eef_yaw_offset, (0, 0, 1))
        quat = transformations.quaternion_multiply(quat, offquat)
        quat = transformations.unit_vector(quat)
        # Set grasp orientation
        msg.grasp_pose.pose.orientation.x = quat[0]
        msg.grasp_pose.pose.orientation.y = quat[1]
        msg.grasp_pose.pose.orientation.z = quat[2]
        msg.grasp_pose.pose.orientation.w = quat[3]
        #rospy.loginfo("*** MoveIt pick pose/tool0 ")

        # Set pre-grasp approach
        msg.pre_grasp_approach.direction.header = header
        msg.pre_grasp_approach.direction.vector = grasp_config.approach
        msg.pre_grasp_approach.min_distance = approach_distance / 2
        msg.pre_grasp_approach.desired_distance = approach_distance

        # Set post-grasp retreat
        msg.post_grasp_retreat.direction.header = header
        msg.post_grasp_retreat.direction.vector.x = -grasp_config.approach.x
        msg.post_grasp_retreat.direction.vector.y = -grasp_config.approach.y
        msg.post_grasp_retreat.direction.vector.z = -grasp_config.approach.z
        msg.post_grasp_retreat.min_distance = approach_distance / 2
        msg.post_grasp_retreat.desired_distance = approach_distance

        res.append(msg)

    return res


if __name__ == '__main__':
    rospy.init_node('pick_and_place_server')
    paps = PickAndPlaceServer()
    rospy.spin()
