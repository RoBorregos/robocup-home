#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import time
import tf

# PICK_GROUPS: [arm] #, whole_body, whole_body_rotational
# ARM_GROUP: "arm"
# ARM_JOINTS: [joint1, joint2, joint3, joint4, joint5, joint6]

import face_recognition
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

CAMERA_TOPIC = "/zed2/zed_node/rgb/image_rect_color"

# Load images
random = face_recognition.load_image_file("known_people/random.png")

# Encodings
random_encodings = face_recognition.face_encodings(random)[0]


# Name people and encodings
people = [
    [random_encodings, "random"]
]
people_encodings = [
    random_encodings
]
people_names = [
    "random"
]

# Make encodings of known people images
folder = "known_people"
def process_imgs():
    for filename in os.listdir(folder):
        if filename == ".DS_Store":
            continue
        
        process_img(filename)


def process_img(filename):
    img = face_recognition.load_image_file(f"{folder}/{filename}")
    cur_encodings = face_recognition.face_encodings(img)

    if len(cur_encodings) == 0:
        print('no encodings found')
        return
    
    if len(cur_encodings) > 0:
        cur_encodings = cur_encodings[0]

    people_encodings.append(cur_encodings)
    people_names.append(filename[:-4])
    people.append([cur_encodings, filename[:-4]])

    print(f"{folder}/{filename}")

process_imgs()

class PlanningNode():
    ARM_GROUP = "arm"
    ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    ARM_CALIBRATION = [-1.57, 0.0, -3.1416 / 4, 0, -3.1416 / 4, -2.356]
    ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ARM_RECOGNITION = [-1.7503315210342407, -1.2894670963287354, -0.7641545534133911, -0.07917511463165283, -0.36656704545021057, 0.8473416566848755]

    def __init__(self):
        rospy.init_node('planning_node')
        self.pick_group = moveit_commander.MoveGroupCommander(PlanningNode.ARM_GROUP, wait_for_servers = 0)
        self.pick_group.set_goal_orientation_tolerance(0.11)
        self.pick_group.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.image = None
        self.image_sub = rospy.Subscriber("/cameras/cam1/image_raw", Image, self.image_callback)
        self.move_arm(PlanningNode.ARM_RECOGNITION)
        
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def plan(self, pose):
        self.pick_group.set_pose_target(pose)
        self.pick_group.plan()
        self.pick_group.go(wait=True)
        self.pick_group.stop()
        self.pick_group.clear_pose_targets()

    def run(self):
        while rospy.is_shutdown() == False :
            print('hola')
            print('state', self.pick_group.get_current_joint_values())
            state = self.pick_group.get_current_joint_values()
            time.sleep(1)
            input_left_or_right = input('left or right?')
            ten_degrees = 0.174533
            if input_left_or_right == 'l':
                print('left')
                # state[0] += 0.1 euler degrees
                
                state[0] += ten_degrees
            else:
                print('right')
                state[0] -= ten_degrees
            print('calibración')
            self.move_arm(state)
            time.sleep(1) 
            print('home')
            # self.move_arm(PlanningNode.ARM_HOME)
            time.sleep(1)
            # self.listener.waitForTransform('Base', 'Cam1', rospy.Time(0), rospy.Duration(5.0))
            # (trans, rot) = self.listener.lookupTransform('Base', 'Cam1', rospy.Time(0))
            # rospy.sleep(1)
            # # move to the camera position in x by 0.1
            # print('hola ', trans, ' ', rot)
            # new_cam_pose = PoseStamped()
            # new_cam_pose.header.frame_id = "Base"
            # new_trans = trans
            # new_trans[0] += 0.1
            # new_cam_pose.pose.position = Point()
            # new_cam_pose.pose.position.x = new_trans[0]
            # new_cam_pose.pose.position.y = new_trans[1]
            # new_cam_pose.pose.position.z = new_trans[2]
            # new_cam_pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])

            # self.plan(new_cam_pose)
            # rospy.sleep(1)
            # print('llegue')

    def move_arm(self, joint_values):
        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = joint_values
        # set speed
        self.pick_group.set_max_velocity_scaling_factor(0.1)
        self.pick_group.go(joint_state, wait=True)
        self.pick_group.stop()

p = PlanningNode()
p.run() 

