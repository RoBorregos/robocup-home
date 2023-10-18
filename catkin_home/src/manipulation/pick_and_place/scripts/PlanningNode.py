#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import time
import tf

import cv2
import face_recognition
import numpy as np
import os

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

i = 0
xc = 0
yc = 0
area = 0
center = [1920/2, 1080/2]
best_area = 185000

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
    
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def plan(self, pose):
        self.pick_group.set_pose_target(pose)
        self.pick_group.plan()
        self.pick_group.go(wait=True)
        self.pick_group.stop()
        self.pick_group.clear_pose_targets()

    def run(self):
        process_this_frame = True
        while rospy.is_shutdown() == False :


            if self.image is not None:
                frame = self.image
                print('image')

                if process_this_frame:
                    # Resize frame of video to 1/4 size for faster face recognition processing
                    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                    
                    # Find all the faces and face encodings in the current frame of video
                    face_locations = face_recognition.face_locations(small_frame)
                    face_encodings = face_recognition.face_encodings(small_frame, face_locations)

                    # Check each encoding found
                    face_names = []
                    for face_encoding in face_encodings:

                        # See if the face is a match for the known face(s)
                        matches = face_recognition.compare_faces(face_encoding, people_encodings, 0.6)
                        name = "Unknown"

                        face_distances = face_recognition.face_distance(people_encodings, face_encoding)
                        best_match_index = np.argmin(face_distances)

                        if matches[best_match_index]:
                            name = people_names[best_match_index]
                        
                            
                        face_names.append(name)

                # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4
                
            

                if process_this_frame and name == "Unknown":
                    print("Unknown")
                    
                    left = max(left - 50,0)
                    right = right + 50
                    top = max(0,top - 50)
                    bottom = bottom + 50
                    result = frame[top:bottom, left:right]
                    new_name = input("Enter name: ")

                    new_name = f"{new_name}{i}.png"
                    new_dir = f"{folder}/{new_name}"
                    cv2.imwrite(new_dir,result)
                    process_img(new_name)

                
                    
                    print(new_name)
                    
                    i = i+1

                # Draw a box around the face
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                xc = left + (right - left)/2
                yc = top + (top - bottom)/2
                area = (right-left)*(bottom-top)

                # Draw a label with a name below the face
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                
            process_this_frame = not process_this_frame
    
            difx = xc - center[0] 
            dify = center[1] - yc
            max_degree = 30

            # print(area)
            move_x = difx*max_degree/center[0]
            move_y = dify*max_degree/center[1]
            print(move_x, ", ",move_y)

            cv2.imshow('Video', frame)
            # cv2.waitKey(1)
            # Hit 'q' on the keyboard to quit!
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            state = self.pick_group.get_current_joint_values()

            if move_x > 0:
                state[0] += move_x*3.1416/180
            else:
                state[0] -= move_x*3.1416/180


            print('hola')
            print('state', self.pick_group.get_current_joint_values())
            time.sleep(1)
            # input_left_or_right = input('left or right?')
            # ten_degrees = 0.174533
            # if input_left_or_right == 'l':
            #     print('left')
            #     # state[0] += 0.1 euler degrees
                
            #     state[0] += ten_degrees
            # else:
            #     print('right')
            #     state[0] -= ten_degrees
            # print('calibración')
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

