#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
import tf.transformations as transformations
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, Grasp, GripperTranslation, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation, AllowedCollisionEntry, PlanningSceneComponents
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal, PickAndPlaceResult, PickAndPlaceFeedback
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import Header, Float64
from gpd_ros.msg import GraspConfig, GraspConfigList
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import numpy
import math
import copy
import time
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class PickAndPlaceServer(object):
    ARM_GROUP = "whole_body"
    PICK_GROUPS = ["whole_body"]
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
        PickAndPlaceServer.PICK_GROUPS = rospy.get_param("PICK_GROUPS", PickAndPlaceServer.PICK_GROUPS)
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
        self.apply_scene_srv = rospy.ServiceProxy(
            '/apply_planning_scene', ApplyPlanningScene)
        self.scene_srv.wait_for_service(timeout=100)
        self.apply_scene_srv.wait_for_service(timeout=100)
        rospy.loginfo("Connected.")
        print("Current allowed collisions:")
        self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
        # self.curr_scene.allowed_collision_matrix.entry_names.append("<octomap>")
        # for entry in self.curr_scene.allowed_collision_matrix.entry_values:
        #     entry.enabled.append(True)
        # self.curr_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True for i in range(len(self.curr_scene.allowed_collision_matrix.entry_names))]))
        self.curr_collision_matrix = self.curr_scene.allowed_collision_matrix
        self.apply_scene_srv(scene=self.curr_scene)
        # self.curr_collision_matrix = self.scene_srv(PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)).scene.allowed_collision_matrix
        # self.curr_collision_matrix.entry_names.append("<octomap>")
        # for entry in self.curr_collision_matrix.entry_values:
        #     entry.enabled.append(True)
        # self.curr_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True for i in range(len(self.curr_collision_matrix.entry_names))]))
        
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
        self.pose_p = rospy.Publisher("planning_pose", PoseStamped, queue_size=10)
        self.pose_debug = rospy.Publisher("pose_debug", PoseStamped, queue_size=10)

        rospy.loginfo("Waiting for MoveGroupCommander...")
        self.pick_groups = []
        for group in PickAndPlaceServer.PICK_GROUPS:
            self.pick_groups.append(moveit_commander.MoveGroupCommander(group, wait_for_servers = 0))
            self.pick_groups[-1].set_goal_orientation_tolerance(0.11)
            self.pick_groups[-1].set_goal_position_tolerance(0.01)

        self.picked_object_dimensions = [0, 0, 0]
        self.place_height = 0.3
        self.pick_height = 0

        self.pub_pick_width = rospy.Publisher("/pick_width", Float64, queue_size=1)

        rospy.loginfo("Action Server Pick & Place Up!")

    def testPose(self):
        t = PoseStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.pose.position.x = -0.10729249562597369
        t.pose.position.y = 0.25381871209802137
        t.pose.position.z = 0.5271093439390989
        t.pose.orientation.x = 0.9103444808785343
        t.pose.orientation.y = 0.41385133335390073
        t.pose.orientation.z = 3.883465277565029e-06
        t.pose.orientation.w = -3.3049841971737527e-07

        x = PoseArray()
        x.header = t.header
        x.poses.append(t.pose)
        self.pose_array_p.publish(x)
        
        self.pick_groups[0].set_pose_target(t.pose)
        self.pick_groups[0].plan()
        res = self.pick_groups[0].go(wait=True)
        self.pick_groups[0].stop()

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
        for grasp in grasps:
            pS = grasp.grasp_pose
            
            pa.header = pS.header
            pa.poses.append(pS.pose)
        
        self.pose_array_p.publish(pa)

        error_code = -1
        count = 0
        for group in PickAndPlaceServer.PICK_GROUPS:
            rospy.loginfo("Trying to pick object with group: " + PickAndPlaceServer.PICK_GROUPS[count])
            error_code = self.grasp_object(grasps, goal.object_name, links_to_allow_contact, group)
            if error_code == 1:
                break
            count += 1
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
        error_code = -1
        for group in PickAndPlaceServer.PICK_GROUPS:
            error_code = self.place_object(goal.target_pose, goal.object_name, goal.allow_contact_with, group)
            if error_code == 1:
                break
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
    def grasp_object(self, grasps, object_name, allow_contact_with = [], pick_group = "arm"):
        # Confirm if not attached already
        scene_attached_objects = self.scene.get_attached_objects([object_name])
        rospy.loginfo("Attached objects:" + str(scene_attached_objects))
        if object_name in scene_attached_objects:
            return
        
        # Plan to Object with the following steps:
        # - Open Gripper
        # - Move to Object
        # - Close Gripper
        PICK_DEBUG = True
        if PICK_DEBUG:
            # Open Gripper
            rospy.loginfo("Opening gripper")
            self.gripper_group.set_named_target("open")
            self.gripper_group.go(wait=True)
            self.gripper_group.stop()
            rospy.sleep(0.25)


            self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
            old_allowed_collision_matrix = copy.deepcopy(self.curr_scene.allowed_collision_matrix)
            self.curr_scene.allowed_collision_matrix.entry_names.append("current_box")
            for entry in self.curr_scene.allowed_collision_matrix.entry_values:
                entry.enabled.append(True)
            self.curr_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True for i in range(len(self.curr_scene.allowed_collision_matrix.entry_names))]))
            self.apply_scene_srv(scene=self.curr_scene)

            rospy.sleep(0.5)

            # Move to Object
            rospy.loginfo("Planning to object")
            success = False
            pose_stamped_used = None
            for grasp in grasps:
                if success:
                    break
                attempts = 0
                while attempts < 1 and not success:
                    count = 0
                    for group in self.pick_groups:
                        planners = ["ompl"]
                        for planner in planners:
                            rospy.loginfo("Trying to pick object with group: " + PickAndPlaceServer.PICK_GROUPS[count])
                            
                            pose_st = PoseStamped()
                            pose_st.header = grasp.grasp_pose.header
                            pose_st.pose = grasp.grasp_pose.pose
                            self.pose_p.publish(pose_st)
                            grasp.grasp_pose.pose.position.z += 0.045
                            adjust = 0.00
                            if grasp.grasp_pose.pose.position.x > 0.0:
                                grasp.grasp_pose.pose.position.x -= adjust
                            else:
                                grasp.grasp_pose.pose.position.x += adjust
                            # same with y
                            if grasp.grasp_pose.pose.position.y > 0.0:
                                grasp.grasp_pose.pose.position.y -= adjust
                            else:
                                grasp.grasp_pose.pose.position.y += adjust
                            group.set_pose_target(grasp.grasp_pose.pose)
                            group.set_goal_orientation_tolerance(numpy.deg2rad(5))
                            group.set_goal_position_tolerance(0.012)
                            group.set_max_velocity_scaling_factor(0.25)
                            group.set_max_acceleration_scaling_factor(0.025)
                            group.set_planning_pipeline_id(planner)
                            # set planner to kpiece
                            group.set_planner_id("RRTConnect")
                            # set planning threads
                            group.set_planning_time(25)
                            group.set_num_planning_attempts(10)

                            t = time.time()
                            res = group.plan()
                            rospy.loginfo("Planning time 1: " + str(time.time() - t))
                            if res[0] == False:
                                rospy.loginfo("Planning failed")
                                continue
                            t = time.time()
                            res = group.go(wait=True)
                            rospy.loginfo("Execution time 1: " + str(time.time() - t))
                            
                            group.stop()
                            rospy.loginfo("Result: " + str(res))
                            if res != True:
                                break
                            
                            adjust = 0
                            if grasp.grasp_pose.pose.position.x > 0.0:
                                grasp.grasp_pose.pose.position.x -= adjust
                            else:
                                grasp.grasp_pose.pose.position.x += adjust
                            # same with y
                            if grasp.grasp_pose.pose.position.y > 0.0:
                                grasp.grasp_pose.pose.position.y -= adjust
                            else:
                                grasp.grasp_pose.pose.position.y += adjust

                            grasp.grasp_pose.pose.position.z -= 0.065

                            self.pick_height = grasp.grasp_pose.pose.position.z

                            pose_st = PoseStamped()
                            pose_st.header = grasp.grasp_pose.header
                            pose_st.pose = grasp.grasp_pose.pose
                            self.pose_p.publish(pose_st)
                            group.set_pose_target(grasp.grasp_pose.pose)
                            group.set_goal_orientation_tolerance(numpy.deg2rad(5))
                            group.set_goal_position_tolerance(0.02)
                            group.set_max_velocity_scaling_factor(0.15)
                            group.set_max_acceleration_scaling_factor(0.025)
                            group.set_planning_pipeline_id(planner)
                            # disable path simplification
                            group.set_planner_id("RRTConnect")
                            # set planning threads
                            group.set_planning_time(25)
                            group.set_num_planning_attempts(10)

                            t = time.time()
                            res = group.plan()
                            rospy.loginfo("Planning time: " + str(time.time() - t))
                            if res[0] == False:
                                rospy.loginfo("Planning failed")
                                continue
                            t = time.time()
                            res = group.go(wait=True)
                            rospy.loginfo("Execution time: " + str(time.time() - t))
                            
                            group.stop()
                            rospy.loginfo("Result: " + str(res))
                            if res == True:
                                pose_stamped_used = pose_st
                                success = True
                                break

                        if success:
                            break
                        count += 1
                    attempts += 1

            if not success:
                # Restore scene
                self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
                self.curr_scene.allowed_collision_matrix = old_allowed_collision_matrix
                self.apply_scene_srv(scene=self.curr_scene)
                return 99999
            

            self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
            self.curr_scene.allowed_collision_matrix.entry_names.append("current")
            for entry in self.curr_scene.allowed_collision_matrix.entry_values:
                entry.enabled.append(True)
            self.curr_scene.allowed_collision_matrix.entry_values.append(AllowedCollisionEntry(enabled=[True for i in range(len(self.curr_scene.allowed_collision_matrix.entry_names))]))
            self.apply_scene_srv(scene=self.curr_scene)

            # Close Gripper
            attempts = 0
            success = False
            while attempts < 5:
                rospy.loginfo("Closing gripper")
                self.gripper_group.set_named_target("close")
                res = self.gripper_group.go(wait=True)
                self.gripper_group.stop()
                attempts += 1
                if res == True:
                    success = True
                    break
                    

            
            # Restore scene
            self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
            self.curr_scene.allowed_collision_matrix = old_allowed_collision_matrix
            self.apply_scene_srv(scene=self.curr_scene)
            

            if success:
                rospy.loginfo("Attaching object to gripper")
                self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
                
                collision_mesh = self.scene.get_objects(["current"])["current"]

                # Attach an Enclosing Box of the mesh
                box_factor = 1.1
                vertices = collision_mesh.meshes[0].vertices
                minPt = Point(vertices[0].x, vertices[0].y, vertices[0].z)
                maxPt = Point(vertices[0].x, vertices[0].y, vertices[0].z)
                for vertex in vertices:
                    if vertex.x < minPt.x:
                        minPt.x = vertex.x
                    if vertex.y < minPt.y:
                        minPt.y = vertex.y
                    if vertex.z < minPt.z:
                        minPt.z = vertex.z
                    if vertex.x > maxPt.x:
                        maxPt.x = vertex.x
                    if vertex.y > maxPt.y:
                        maxPt.y = vertex.y
                    if vertex.z > maxPt.z:
                        maxPt.z = vertex.z
                
                width = maxPt.x - minPt.x
                height = maxPt.y - minPt.y
                depth = maxPt.z - minPt.z
                self.picked_object_dimensions = [width, height, depth]

                # heigh for place
                self.place_height = depth #option 1
                self.place_height = self.pick_height #option 2
                self.pub_pick_width.publish(width)

                rospy.loginfo("Object dimensions: " + str(self.picked_object_dimensions))
                collision_object = CollisionObject(
                    header = collision_mesh.header,
                    id = "current_box",
                    pose = collision_mesh.pose,
                    primitives = [SolidPrimitive(
                        type=SolidPrimitive.BOX,
                        dimensions=[width * box_factor, height * box_factor, depth * box_factor]
                    )],
                    primitive_poses = [collision_mesh.mesh_poses[0]],
                    operation = CollisionObject.ADD
                )



                self.curr_scene.robot_state.attached_collision_objects.append(
                    AttachedCollisionObject(
                        link_name="Base_Gripper",
                        object=collision_object,
                        touch_links=["Base_Gripper", "Servo1", "Servo2", "Dedo1", "Dedo2", "link1"],
                        detach_posture= JointTrajectory(
                            joint_names=["Rev1Servo", "Rev2Servo"],
                            points=[JointTrajectoryPoint(
                                positions=[-0.50, 0.50],
                                time_from_start=rospy.Duration(0.5)
                            )]
                        ),
                        weight=0.0
                    )
                )

                # Remove Current
                self.scene.remove_world_object("current")
                self.scene.remove_world_object("current_box")
                rospy.sleep(0.2)

                self.apply_scene_srv(scene=self.curr_scene)
                rospy.loginfo("Attached object to gripper")


                """pose_st = PoseStamped()
                pose_st.header = grasp.grasp_pose.header
                pose_st.pose = grasp.grasp_pose.pose
                pose_st.pose.position.z += 0.08
                self.pose_p.publish(pose_st)
                grasp.grasp_pose.pose.position.z += 0.08
                group.set_pose_target(grasp.grasp_pose.pose)
                group.set_goal_orientation_tolerance(numpy.deg2rad(5))
                group.set_goal_position_tolerance(0.012)
                group.set_max_velocity_scaling_factor(0.10)
                group.set_max_acceleration_scaling_factor(0.001)
                group.set_planning_pipeline_id("ompl")
                group.set_planning_time(3)
                res = group.plan()
                res = group.go(wait=True)
                group.stop()"""

                # restore scene to detect collisions
                rospy.loginfo("Restoring scene")
                self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
                self.curr_scene.allowed_collision_matrix = old_allowed_collision_matrix
                self.apply_scene_srv(scene=self.curr_scene)
                
                return 1

            if not success:
                return 99999

        else:
            possible_grasps = grasps

            goal = createPickupGoal(
                pick_group, object_name, possible_grasps, allow_contact_with, "<octomap>" , None)
            
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
    def place_object(self, object_pose, object_name, allow_contact_with = [], place_group = "arm"):

        links_to_allow_contact = PickAndPlaceServer.ALLOW_CONTACT
        links_to_allow_contact.extend(allow_contact_with)
        
        #possible_placings = create_placings_from_object_pose(object_pose, links_to_allow_contact)

        error_code = -1
        
        for group in self.pick_groups:
            planners = ["ompl"]
            for planner in planners:
                rospy.loginfo("Trying to place with arm and torso")
                                    
                pose_st = PoseStamped()
                pose_st.header = object_pose.header
                pose_st.pose = object_pose.pose
                #pose_st.pose.position.z += self.picked_object_dimensions[2] + 0.07
                rospy.loginfo("Planning with object added height: " + str(self.place_height))
                pose_st.pose.position.z += self.place_height + 0.07
                #pose_st.pose.position.y -= 0.05
                #face down
                pose_st.pose.orientation = Quaternion(x=0.000001, y=0.707000, z=0.000001, w=0.707000)
                self.pose_p.publish(pose_st)
                group.set_pose_target(pose_st.pose)
                group.set_goal_orientation_tolerance(numpy.deg2rad(5))
                group.set_goal_position_tolerance(0.012)
                group.set_max_velocity_scaling_factor(0.2)
                group.set_max_acceleration_scaling_factor(0.025)
                group.set_planning_pipeline_id(planner)
                # set planner to kpiece
                group.set_planner_id("RRTConnect")
                # set planning threads
                group.set_planning_time(25)
                group.set_num_planning_attempts(10)

                self.pose_debug.publish(pose_st)
                print("Planning to pose: " + str(pose_st.pose))
                
                t = time.time()
                res = group.plan()
                rospy.loginfo("Planning time 1: " + str(time.time() - t))
                if res[0] == False:
                    rospy.loginfo("Planning failed")
                    continue
                t = time.time()
                res = group.go(wait=True)
                rospy.loginfo("Execution time 1: " + str(time.time() - t))
                
                group.stop()
                rospy.loginfo("Result: " + str(res))
                if res != True:
                    break
            
                attempts = 0
                success = False
                while attempts < 5:
                    rospy.loginfo("Opening gripper")
                    self.gripper_group.set_named_target("open")
                    res = self.gripper_group.go(wait=True)
                    self.gripper_group.stop()
                    attempts += 1
                    if res == True:
                        success = True
                        break

                # restore scene to detect collisions
                rospy.loginfo("Restoring scene")
                self.curr_scene = self.scene_srv(PlanningSceneComponents()).scene
                # detach object box named current_box
                self.curr_scene.robot_state.attached_collision_objects = []
                # Remove Current
                self.scene.remove_world_object("current")
                self.scene.remove_world_object("current_box")

                self.apply_scene_srv(scene=self.curr_scene)
                

                place_success = True 
                
                return 1
    
    # Function to publish the 3D point as a marker
                """marker = Marker()
                marker.header.frame_id = "world"  # Replace with your actual world frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "my_namespace"
                marker.id = 0
                # marker as arrow pointing down
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.position.x = point_world.point.x
                marker.pose.position.y = point_world.point.y
                marker.pose.position.z = point_world.point.z + 0.1
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.707
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 0.707
                marker.scale.x = 0.1
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                self._marker_pub.publish(marker)"""

        
        """goal = createPlaceGoal(
            possible_placings, place_group, object_name, links_to_allow_contact, "<octomap>", None)

        error_code = self.handle_place_as(goal)
    
        # Confirm STATUS looking into attached objects
        error_code = self.confirm_status_with_attached_objects(error_code, object_name, False)

        rospy.logwarn(
            "Place Result: " +
        str(moveit_error_dict[error_code]))"""

        return 0


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
    pug.allow_gripper_support_collision = False
    pug.allowed_planning_time = 30.0
    pug.planning_options.planning_scene_diff.is_diff = True
    pug.planning_options.planning_scene_diff.robot_state.is_diff = True
    pug.planning_options.plan_only = False
    pug.planning_options.replan = True
    pug.planning_options.replan_attempts = 3
    pug.allowed_touch_objects = []
    pug.allowed_touch_objects.extend(links_to_allow_contact)
    pug.attached_object_touch_links = []
    pug.attached_object_touch_links.extend(links_to_allow_contact)
    # pug.planning_options.planning_scene_diff.allowed_collision_matrix = curr_collision_matrix
    
    # pug.path_constraints = Constraints()
    # pug.path_constraints.position_constraints = []
    # pug.path_constraints.orientation_constraints = []

    # position_constraints = PositionConstraint()
    # position_constraints.link_name = "end_effector_link"
    # position_constraints.header.frame_id = "end_effector_link"
    # position_constraints.target_point_offset = Vector3(x=0.03, y=0.03, z=0.03)
    # position_constraints.weight = 1.0
    # pug.path_constraints.position_constraints.append(position_constraints)

    # orientation_constraints = OrientationConstraint()
    # orientation_constraints.link_name = "end_effector_link"
    # orientation_constraints.header.frame_id = "base_link"
    # orientation_constraints.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    # orientation_constraints.absolute_x_axis_tolerance = numpy.deg2rad(10)
    # orientation_constraints.absolute_y_axis_tolerance = numpy.deg2rad(10)
    # orientation_constraints.absolute_z_axis_tolerance = numpy.deg2rad(10)
    # orientation_constraints.weight = 1.0
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

    gripper_pre_grasp_positions = PickAndPlaceServer.GRIPPER_OPEN
    gripper_grasp_positions = PickAndPlaceServer.GRIPPER_CLOSED
    fix_tool_frame_to_grasping_frame_roll = 0.0
    fix_tool_frame_to_grasping_frame_pitch = 0.0
    fix_tool_frame_to_grasping_frame_yaw = 0.0
    grasp_desired_distance = 0.07
    grasp_min_distance = 0.03
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

    # g.pre_grasp_posture = pre_grasp_posture
    # g.grasp_posture = grasp_posture

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

    g.grasp_pose = poseStamped # PoseStamped(header, fixed_pose)
    g.grasp_quality = grasp_config.score.data # min(1000, max(0, grasp_quality)) / 1000

    # g.pre_grasp_approach = createGripperTranslation(
    #     Vector3(1.0, 0.0, 0.0), desired_distance = grasp_desired_distance,
    #     min_distance = grasp_min_distance)
    
    # g.post_grasp_retreat = createGripperTranslation(
    #     Vector3(-1.0, -0.0, 0.0), desired_distance = grasp_desired_distance,
    #     min_distance = grasp_min_distance)

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
    header.frame_id = "Base"
    kThresholdScore = -1000.0
    res = []
    for grasp_config in grasp_config_list.grasps:
        if grasp_config.score.data < kThresholdScore:
                rospy.loginfo("Grasp Ignored, It's score is %f", grasp_config.score.data)
                continue
        rospy.loginfo("grasp score is %f", grasp_config.score.data)
        rospy.loginfo("grasp required width %f", grasp_config.width.data)

        # Set grasp position, translation from hand-base to the parent-link of EEF
        grasp_pose = PoseStamped()
        grasp_pose.header = header
        
        grasp_pose.pose.position.x = grasp_config.position.x
        grasp_pose.pose.position.y = grasp_config.position.y
        grasp_pose.pose.position.z = grasp_config.position.z # + 0.40

        # Rotation Matrix
        rot = numpy.array([[grasp_config.approach.x, grasp_config.binormal.x, grasp_config.axis.x],
                        [grasp_config.approach.y, grasp_config.binormal.y, grasp_config.axis.y],
                        [grasp_config.approach.z, grasp_config.binormal.z, grasp_config.axis.z]])
        R = numpy.eye(4)
        R[:3, :3] = rot
        quat = transformations.quaternion_from_matrix(R)

        # EEF yaw-offset to its parent-link (last link of arm)
        # eef_yaw_offset = 0.0
        # offquat = transformations.quaternion_about_axis(eef_yaw_offset, (0, 0, 1))
        # quat = transformations.quaternion_multiply(quat, offquat)
        # quat = transformations.unit_vector(quat)

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
