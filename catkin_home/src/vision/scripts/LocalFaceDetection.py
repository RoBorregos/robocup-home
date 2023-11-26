
#!/usr/bin/env python3
import time
import tf

import cv2
import numpy as np
import os

import face_recognition

CAMERA_TOPIC = '/zed2/zed_node/rgb/image_rect_color'

TRACK_THRESHOLD = 70
AREA_THRESHOLD = 10000 #120000

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


class FaceDetection():

    def __init__(self):
        self.image = None
        self.result_img = None

        process_imgs()

        self.run()

    def run(self):
        process_this_frame = True
        x, y, w, h = 0, 0, 0, 0
        i = 0
        xc = 0
        yc = 0
        center = [960/2,540/2] #[1920/2, 1080/2]
        cap = cv2.VideoCapture(0)

        while True:

            xc = 960/2
            yc = 540/2

            # print("loop")
            detected_faces = []

            if True:
                _, frame = cap.read()
                center = [frame.shape[1]/2, frame.shape[0]/2]
                cv2.imshow('frame', frame)
                #frame = cv2.flip(frame, 0)
                # print('image')

                # Resize frame of video to 1/4 size for faster face recognition processing
                small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
                
                # Find all the faces and face encodings in the current frame of video
                face_locations = face_recognition.face_locations(small_frame)
                face_encodings = face_recognition.face_encodings(small_frame, face_locations)

                xc = 0
                yc = 0

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


                    # Display the results
                for (top, right, bottom, left), name in zip(face_locations, face_names):
                    top *= 4
                    right *= 4
                    bottom *= 4
                    left *= 4

                    area = (right-left)*(bottom-top)
                
                    if process_this_frame and name[0] == "Unknown" and area > AREA_THRESHOLD:
                        # print("Unknown")
                        # cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                        
                        left = max(left - 50,0)
                        right = right + 50
                        top = max(0,top - 50)
                        bottom = bottom + 50
                        result = frame[top:bottom, left:right]

                        # new_name = input("Enter name: ")
                        new_name = "face"
                        name[0] = new_name
                        new_name = f"{new_name}{i}.png"
                        new_dir = f"{folder}/{new_name}"
                        cv2.imwrite(new_dir,result)
                        process_img(new_name)
                        
                        # print(name[0])
                        
                        i = i+1
    
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
                    cv2.imshow('img', frame)
            
                

                if xc != 0:
                    difx = xc - center[0] 
                
                else:
                    difx = 0

                if yc != 0:
                    dify = center[1] - yc
                else:
                    dify = 0
    
                max_degree = 30

                # print(area)
                move_x = difx*max_degree/center[0]
                move_y = dify*max_degree/center[1]


                # img_arr = img_list()

                # for i in range(len(imgs)):
                #     img_arr.images.append(img())
                #     img_arr.images[i].x = int(imgs[i].x)
                #     img_arr.images[i].y = int(imgs[i].y)
                #     img_arr.images[i].w = int(imgs[i].w)
                #     img_arr.images[i].h = int(imgs[i].h)
                #     img_arr.images[i].name = imgs[i].name


                # img_arr.images = imgs
                # img_arr.len = len(imgs)

                if move_x != 0 or move_y != 0:
                    print(move_x, ", ",move_y)
                    #self.move_pub.publish(moveMsg)

                # print(move_x, ", ",move_y)
        # state = self.pick_group.get_current_joint_values()

        # if move_x > 0:img_arr
        #     state[0] += move_x*3.1416/180
        # else:
        #     state[0] -= move_x*3.1416/180
            process_this_frame = not process_this_frame



if __name__ == '__main__':
    FaceDetection()
    cv2.destroyAllWindows()