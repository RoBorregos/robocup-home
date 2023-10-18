import cv2
import face_recognition
import numpy as np
import os

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


# Capture video from webcam
cap = cv2.VideoCapture(1)
i = 0
xc = 0
yc = 0
area = 0
center = [1920/2, 1080/2]
best_area = 185000

# Only process one frame out of every 2
process_this_frame = True

while(True):
    ret, frame = cap.read()
    
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

    print(area)
    

    print(difx*max_degree/center[0], ", ", dify*max_degree/center[1])

   # print(xc, ", ", yc)
   
    # Display the resulting image
    cv2.imshow('Video', frame)
    # cv2.waitKey(1)
    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 1080, 1920


cap.release()
cv2.destroyAllWindows()
print("Hello World!")

