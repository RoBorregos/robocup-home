#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf
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

    def __init__(self):
        self.pick_group = moveit_commander.MoveGroupCommander(PlanningNode.ARM_GROUP, wait_for_servers = 0)
        self.pick_group.set_goal_orientation_tolerance(0.11)
        self.pick_group.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.image = None
        self.image_sub = rospy.Subscriber("/cameras/cam1/image_raw", Image, self.image_callback)
        rospy.init_node('planning_node')
    
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def plan(self, pose):
        self.pick_group.set_pose_target(pose)
        self.pick_group.plan()
        self.pick_group.go(wait=True)
        self.pick_group.stop()
        self.pick_group.clear_pose_targets()

    def run(self):
        self.listener.waitForTransform('Base', 'Cam1', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('Base', 'Cam1', rospy.Time(0))
        # move to the camera position in x by 0.1
        print('hola ', trans, ' ', rot)
        new_cam_pose = PoseStamped()
        new_cam_pose.header.frame_id = "Base"

p = PlanningNode()
p.run() 

