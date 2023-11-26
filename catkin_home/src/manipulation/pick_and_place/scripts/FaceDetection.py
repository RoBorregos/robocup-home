#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import time
import tf

import cv2
import numpy as np
import os

import face_recognition
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pick_and_place.msgs import img, img_list, move

CAMERA_TOPIC = "/zed2/zed_node/rgb/image_rect_color"

TRACK_THRESHOLD = 70
AREA_THRESHOLD = 120000

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

class FaceDetection():

    def __init__(self):
        rospy.init_node('face_detection')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_callback)
        self.detection_pub = rospy.Publisher("detection_results", img_list, queue_size=1)
        self.move_pub = rospy.Publisher("move", move, queue_size=1)
        self.image = None

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def run(self):
        process_this_frame = True
        x, y, w, h = 0, 0, 0, 0

        while rospy.is_shutdown() == False :


            img_arr = img_list()

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
                    for face_encoding, location in zip(face_encodings, face_locations):
                        # print("l____",location[0])
            
                        flag = False
                        print("detected: ", len(detected_faces))
                        for detected in detected_faces:
                            centerx = (location[3] + (location[1] - location[3])/2)*4
                            centery = (location[0] + (location[0] - location[2])/2)*4

                            # print("detected: ", detected["y"], "center: ", centery, "diff: ", abs(detected["y"] - centery))

                            if (abs(detected["x"] - centerx) < TRACK_THRESHOLD) and (abs(detected["y"] - centery) < TRACK_THRESHOLD):
                                name = detected["name"]
                                flag = True
                                # print("same")
                                break

                            # x = 50, new x = 55, 50-55 = 5, 
                        if not flag:
                            name = "Unknown"

                        # See if the face is a match for the known face(s)
                            matches = face_recognition.compare_faces(face_encoding, people_encodings, 0.6)
                            

                            face_distances = face_recognition.face_distance(people_encodings, face_encoding)
                            best_match_index = np.argmin(face_distances)

                            if matches[best_match_index]:
                                name = people_names[best_match_index]
                        
                            
                        face_names.append([name,flag])
                        
                detected_faces = []

                    # for face_encoding in face_encodings:

                    #     # See if the face is a match for the known face(s)
                    #     matches = face_recognition.compare_faces(face_encoding, people_encodings, 0.6)
                    #     name = "Unknown"

                    #     face_distances = face_recognition.face_distance(people_encodings, face_encoding)
                    #     best_match_index = np.argmin(face_distances)

                    #     if matches[best_match_index]:
                    #         name = people_names[best_match_index]
                        
                            
                    #     face_names.append(name)

                # Display the results
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                imgbb = img()
                imgbb.x = top
                imgbb.y = left
                imgbb.w = right - left
                imgbb.h = bottom - top
                imgbb.name = name

                img_arr.append(imgbb)
                area = (right-left)*(bottom-top)
            
                if process_this_frame and name[0] == "Unknown" and area > AREA_THRESHOLD:
                    # print("Unknown")
                    # cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                    
                    left = max(left - 50,0)
                    right = right + 50
                    top = max(0,top - 50)
                    bottom = bottom + 50
                    result = frame[top:bottom, left:right]

                    new_name = input("Enter name: ")
                    name[0] = new_name
                    new_name = f"{new_name}{i}.png"
                    new_dir = f"{folder}/{new_name}"
                    cv2.imwrite(new_dir,result)
                    process_img(new_name)
                    
                    # print(name[0])
                    
                    i = i+1
                # if process_this_frame and name == "Unknown":
                #     print("Unknown")
                    
                #     left = max(left - 50,0)
                #     right = right + 50
                #     top = max(0,top - 50)
                #     bottom = bottom + 50
                #     result = frame[top:bottom, left:right]
                #     new_name = input("Enter name: ")

                #     new_name = f"{new_name}{i}.png"
                #     new_dir = f"{folder}/{new_name}"
                #     cv2.imwrite(new_dir,result)
                #     process_img(new_name)

                
                    
                #     print(new_name)
                    
                #     i = i+1
        # Draw a box around the face
        
                xc = left + (right - left)/2
                yc = top + (top - bottom)/2
                area = (right-left)*(bottom-top)
                # print(xc)

                detected_faces.append({"x": xc, "y": yc, "name": name[0]})

                # Draw a label with a name below the face
                if name[1]:
                    cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (255, 0, 0), cv2.FILLED)
                    cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
                else:
                    cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                font = cv2.FONT_HERSHEY_DUPLEX

                cv2.putText(frame, name[0], (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        
                # Draw a box around the face
                # cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                # xc = left + (right - left)/2
                # yc = top + (top - bottom)/2
                # area = (right-left)*(bottom-top)

                # # Draw a label with a name below the face
                # cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                # font = cv2.FONT_HERSHEY_DUPLEX
                # cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                
            process_this_frame = not process_this_frame
    
            difx = xc - center[0] 
            dify = center[1] - yc
            max_degree = 30

            # print(area)
            move_x = difx*max_degree/center[0]
            move_y = dify*max_degree/center[1]

            move = move()
            
            move.x = move_x
            move.y = move_y

            self.move_pub.publish(move)
            self.detection_pub.publish(img_arr)

            print(move_x, ", ",move_y)
            
            # state = self.pick_group.get_current_joint_values()

            # if move_x > 0:
            #     state[0] += move_x*3.1416/180
            # else:
            #     state[0] -= move_x*3.1416/180


        
    
           
           

p = FaceDetection()
p.run() 