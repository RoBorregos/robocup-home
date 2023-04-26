#!/usr/bin/env python3
import rospy
import actionlib
import time
from enum import Enum
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from std_msgs.msg import Bool
from intercom.msg import action_selector_cmd, bring_something_cmd
from object_detector.msg import DetectObjects3DAction, DetectObjects3DGoal, objectDetectionArray, objectDetection
from pick_and_place.msg import PickAndPlaceAction, PickAndPlaceGoal
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf import transformations
from tf.transformations import quaternion_from_euler

class MoveItErrorCodes(Enum):
    SUCCESS = 1
    PLANNING_FAILED = -1
    INVALID_MOTION_PLAN = -2
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
    CONTROL_FAILED = -4
    UNABLE_TO_AQUIRE_SENSOR_DATA = -5
    TIMED_OUT = -6
    PREEMPTED = -7


ARGS= {
    "ENABLE_SPEECH": False,
    "ENABLE_NAVIGATION": False,
    "ENABLE_MANIPULATION": True,
    "ENABLE_VISION": True,
    "TEST_VISION": True,
    "VERBOSE": True,
}

OBJECTS_NAME= {
    1 : 'Coca-Cola',
    2 : 'Coffee',
    3 : 'Nesquik',
    4 : 'Zucaritas',
    5 : 'Harpic'
}
OBJECTS_ID= {
    'Coca-Cola' : 1,
    'Coffee' : 2,
    'Nesquik' : 3,
    'Zucaritas' : 4,
    'Harpic' : 5
}

class ManipulationGoals(Enum):
    COKE = 1
    COFFEE = 2
    NESQUIK = 3
    ZUCARITAS = 4
    HARPIC = 5
    BIGGEST = 6

class CarryMyLuggage(object):
    targetPlace = None
    targetObject = None

    def __init__(self):
        # Conversation/Speech
        if ARGS["ENABLE_SPEECH"]:
            rospy.loginfo("Setting Up Speech")
            self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
            self.parser_listener = rospy.Subscriber('action/bring_something', bring_something_cmd, self.listen_parser)
            self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)
        
        # Toggle Octomap Service
        if ARGS["ENABLE_VISION"] and ARGS["ENABLE_MANIPULATION"]:
            rospy.loginfo("Waiting for /toggle_octomap Service...")
            self.toggle_octomap = rospy.ServiceProxy('/toggle_octomap', SetBool)

        # Mechanisms | Initialize Robot Pose
        if ARGS["ENABLE_MANIPULATION"]:
            rospy.loginfo("Waiting for MoveGroupCommander ARM_TORSO...")
            self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP, wait_for_servers = 0)
            self.initARM()

        # Vision
        if ARGS["ENABLE_VISION"]:
            rospy.loginfo("Setting Up Vision")
            self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)
            rospy.loginfo("Waiting for ComputerVision 3D AS...")
            self.vision3D_as = actionlib.SimpleActionClient("Detect3DFloor", DetectObjects3DAction)
            self.vision3D_as.wait_for_server()
            # 0 - Derecha, 1 - Izquierda, 2 - Ambos

        # Navigation
        if ARGS["ENABLE_NAVIGATION"]:
            rospy.loginfo("Waiting for MoveBase AS...")
            self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.move_client.wait_for_server()
        
        # Testing Vision
        if ARGS["TEST_VISION"]:
            # Test Getting Objects:
            rospy.loginfo("Getting objects")
            found = self.get_object(target = ManipulationGoals['BIGGEST'], side = -1)
            if not found:
                rospy.loginfo("Object Not Found")
                return
            rospy.loginfo("Object Found")
        
        if ARGS["TEST_MANIPULATION"]:
            # Test Getting Objects:
            rospy.loginfo("Getting objects")
            found = self.get_object(target = ManipulationGoals['BIGGEST'], side = -1)
            if not found:
                rospy.loginfo("Object Not Found")
                return
            rospy.loginfo("Object Found")
        
        if ARGS["ENABLE_MANIPULATION"]:
            # Manipulation
            rospy.loginfo("Waiting for /pickup_pose AS...")
            self.pick_as = actionlib.SimpleActionClient('/pickup_pose', PickAndPlaceAction)
            self.pick_as.wait_for_server()
            rospy.loginfo("Waiting for /place_pose AS...")
            self.place_as = actionlib.SimpleActionClient('/place_pose', PickAndPlaceAction)
            self.place_as.wait_for_server()
            self.pick_goal_publisher = rospy.Publisher("pose_pickup/goal", PoseStamped, queue_size=5)
            self.place_goal_publisher = rospy.Publisher("pose_place/goal", PoseStamped, queue_size=5)
        
        rospy.loginfo("Loaded everything...")

    def move_relative(self, x = 0, y = 0, z = 0, o_x = 0, o_y = 0, o_z = 0, w = 1):
        goal = MoveBaseGoal()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = o_x
        pose_stamped.pose.orientation.y = o_y
        pose_stamped.pose.orientation.z = o_z
        pose_stamped.pose.orientation.w = w
        goal.target_pose = pose_stamped
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()

    def moveARM(self, joints):
        if VISION_ENABLE:
            self.toggle_octomap(False)
        ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
        joint_state = JointState()
        joint_state.name = ARM_JOINTS
        joint_state.position = joints
        self.arm_group.go(joint_state, wait=True)
        self.arm_group.stop()
        if VISION_ENABLE:
            self.toggle_octomap(True)

    def initARM(self):
        ARM_INIT = rospy.get_param("ARM_INIT", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.moveARM(ARM_INIT)
    
    def graspARM(self):
        ARM_GRASP = rospy.get_param("ARM_GRASP", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.moveARM(ARM_GRASP)    

    def manipulateObject(self, side = -1):
        target = ManipulationGoals("BIGGEST")

        if not VISION_ENABLE:
            return

        # Get Object:
        rospy.loginfo("Getting object")
        found = self.get_object(target = ManipulationGoals['BIGGEST'], side = -1)
        if not found:
            rospy.loginfo("Object Not Found")
            return
        
        rospy.loginfo("Object Extracted")

        TEST_GDP = False
        in_ = -1
        while TEST_GDP  and not rospy.is_shutdown() and in_ != 0:
            rospy.loginfo("Getting grasping points")
            grasping_points = self.get_grasping_points()
            rospy.loginfo("Grasping Points Found")
            rospy.loginfo("Waiting for user input")
            in_ = handleIntInput("(0) Quit, (1) Retry Same PC", range=(0, 1))

        if not ARGS["ENABLE_MANIPULATION"]:
            return
        
        grasping_points = self.get_grasping_points()
        if grasping_points is None:
            rospy.loginfo("Grasping Points Not Found")
            return

        # Move to Object
        self.toggle_octomap(False)
        self.grasp_config_list.publish(grasping_points)
        rospy.loginfo("Robot Picking " + target.name + " up")
        result = self.pick(self.object_pose, "current", allow_contact_with_ = ["<octomap>"], grasping_points = grasping_points)
        if result != 1:
            rospy.loginfo("Pick Failed")
            return
        rospy.loginfo("Robot Picked " + target.name + " up")
        ## Move Up
        self.object_pose.pose.position.z += 0.05
        self.object_pose.pose.position.x -= 0.1
        self.arm_group.set_pose_target(self.object_pose)
        self.arm_group.plan()
        self.arm_group.go(wait=True)
        self.arm_group.stop()    
        self.toggle_octomap(True)
    
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

    def get_object(self, target = ManipulationGoals['BIGGEST'], side = -1):
        
        class GetObjectsScope:
            success = False
            detection = objectDetection()
            object_pose = []
            object_cloud = []
            object_cloud_indexed = []
            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            result_received = False
        
        def get_objects_feedback(feedback_msg):
            GetObjectsScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            if result is None:
                GetObjectsScope.success = False
                GetObjectsScope.result_received = True
                return
            GetObjectsScope.success = result.success
            GetObjectsScope.object_pose = result.object_pose
            GetObjectsScope.object_cloud = result.object_cloud
            GetObjectsScope.object_cloud_indexed = result.object_cloud_indexed
            GetObjectsScope.x_plane = result.x_plane
            GetObjectsScope.y_plane = result.y_plane
            GetObjectsScope.z_plane = result.z_plane
            GetObjectsScope.width_plane = result.width_plane
            GetObjectsScope.height_plane = result.height_plane
            GetObjectsScope.result_received = True

        if target == ManipulationGoals['BIGGEST']:
            goal = DetectObjects3DGoal(force_object = objectDetectionArray(), side = side, ignore_moveit = ARGS["ENABLE_MANIPULATION"])

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
        self.object_cloud_indexed = GetObjectsScope.object_cloud_indexed

        return GetObjectsScope.success

    def run(self):
        pass


def main():
    try:
        rospy.init_node('CarryMyLuggage', anonymous=True)
        rospy.loginfo("CarryMyLuggage initialized.")
        for key in ARGS:
            ARGS[key] = rospy.get_param('~' + key, ARGS[key])    
        CarryMyLuggage()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
