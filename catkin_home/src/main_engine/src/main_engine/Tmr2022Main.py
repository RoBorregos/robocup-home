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
from object_detector.msg import DetectObjects3DAction, DetectObjects3DGoal  
from object_manipulation.msg import PickUpPoseAction, PickUpPoseGoal
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
    "ENABLE_MANIPULATION": False,
    "ENABLE_VISION3D": False,
    "VERBOSE": True,
}

OBJECTS_NAME= {
    1 : 'Coca-Cola',
    2: 'Coffee',
    3 : 'Nesquik',
}
OBJECTS_ID= {
    'Coca-Cola' : 1,
    'Coffee' : 2,
    'Nesquik' : 3,
}

class Tmr2022Main(object):
    targetPlace = None
    targetObject = None

    def __init__(self):
        # Conversation/Speech
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.parser_listener = rospy.Subscriber('action/bring_something', bring_something_cmd, self.listen_parser)
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)
        
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
    
    def pick_random_object(self):
        if len(self.objects) == 0:
            return -1
        
        targetDetails = self.objects[0]
        
        return self.pick(targetDetails[2], targetDetails[1], [])

    def run(self):
        pass

    def listen_parser(self):
        self.targetPlace = bring_something_cmd.place
        self.targetObject = bring_something_cmd.object
        rospy.loginfo("Parser Received " + bring_something_cmd.place + "-" + bring_something_cmd.object)

    def get_objects(self):
        class GetObjectsScope:
            objects_found_so_far = 0
            objects_found = 0
            objects_poses = []
            objects_names = []
            objects_ids = []

            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            result_received = False
        
        def get_objects_feedback(feedback_msg):
            GetObjectsScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            GetObjectsScope.objects_poses = result.objects_poses
            GetObjectsScope.objects_names = result.objects_names
            GetObjectsScope.objects_found = result.objects_found
            GetObjectsScope.objects_ids = result.objects_ids
            GetObjectsScope.x_plane = result.x_plane
            GetObjectsScope.y_plane = result.y_plane
            GetObjectsScope.z_plane = result.z_plane
            GetObjectsScope.width_plane = result.width_plane
            GetObjectsScope.height_plane = result.height_plane
            GetObjectsScope.result_received = True

        self.vision3D_as.send_goal(
                    DetectObjects3DGoal(),
                    feedback_cb=get_objects_feedback,
                    done_cb=get_result_callback)
        
        start_time = time.time()
        while not GetObjectsScope.result_received:
            pass
        
        object_poses = GetObjectsScope.objects_poses
        object_names = GetObjectsScope.objects_names
        object_ids = GetObjectsScope.objects_ids
        self.objects = list(zip(object_ids, object_names, object_poses))
    
    def lower_head(self):
        joint_goal = self.neck_group.get_current_joint_values()
        joint_goal[0] = math.radians(0.15)
        self.neck_group.go(joint_goal, wait=False)
    
    def raise_head(self):
        rospy.loginfo("Moving head up")
        joint_goal = self.neck_group.get_current_joint_values()
        joint_goal[0] = math.radians(-0.15)
        self.neck_group.go(joint_goal, wait=False)
    
    def center_head(self):
        rospy.loginfo("Moving head to center")
        joint_goal = self.neck_group.get_current_joint_values()
        joint_goal[0] = math.radians(0.0)
        self.neck_group.go(joint_goal, wait=False)

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
    
    def retreat_arm(self):
        rospy.loginfo("Retreating right arm safely")
        joint_goal = self.arm_group.get_current_joint_values()
        # TODO: Define Angles.
        # joint_goal[0] = math.radians(0)
        # joint_goal[1] = math.radians(0) 
        # joint_goal[2] = math.radians(0)
        self.arm_group.go(joint_goal, wait=False)

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
    
def main():
    try:
        rospy.init_node('Tmr2022Main', anonymous=True)
        rospy.loginfo("Tmr2022Main initialized.")
        for key in ARGS:
            ARGS[key] = rospy.get_param('~' + key, ARGS[key])    
        Tmr2022Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
