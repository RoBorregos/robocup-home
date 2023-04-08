import mediapipe as mp
from time import sleep
from typing import Tuple
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

# Calling the pose solution from MediaPipe
mp_pose = mp.solutions.pose

# Calling the solution for image drawing from MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Declaring the CvBridge for image conversion from ROS to OpenCV
bridge = CvBridge()

# Declaring the image and its callback for the ROS topic
imageReceved = None
def image_callback(data):
    global imageReceved
    imageReceved = data

# Initializing the ROS node
rospy.init_node('ImageRecever', anonymous=True)

# Subscribing to the ROS topic
imageSub = rospy.Subscriber(
    "/hsrb/head_center_camera/image_raw", Image, image_callback)

# Publishing the pose estimation results
posePub = rospy.Publisher('/pose_estimation', Point, queue_size=10)

# Calling the pose detection model
with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose:
    # Looping through the image frames
    while not rospy.is_shutdown():
        if imageReceved is not None:
            # Converting the ROS image to OpenCV
            image = bridge.imgmsg_to_cv2(imageReceved, "rgb8")

            # Detecting the pose with the image
            image.flags.writeable = False
            results = pose.process(image)

            # # Drawing the pose detection results
            # image.flags.writeable = True
            # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # mp_drawing.draw_landmarks(
            #     image,
            #     results.pose_landmarks,
            #     mp_pose.POSE_CONNECTIONS,
            #     landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())


            # cv2.imshow('MediaPipe Pose', image)
            # if cv2.waitKey(5) & 0xFF == 27:
            #     break

            if results.pose_landmarks:
                        x = (
                            results.pose_landmarks.landmark[12].x + results.pose_landmarks.landmark[11].x) / 2
                        y = (
                            results.pose_landmarks.landmark[12].y + results.pose_landmarks.landmark[11].y) / 2
                        z = (
                            results.pose_landmarks.landmark[12].z + results.pose_landmarks.landmark[11].z) / 2
                        chest = Point()
                        chest.x = x
                        chest.y = y
                        chest.z = z
                        posePub.publish(chest)
        else:
            print("Image not recived")
        sleep(1)