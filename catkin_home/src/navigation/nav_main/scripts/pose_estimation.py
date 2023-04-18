#!/usr/bin/env python3
import mediapipe as mp
from time import sleep
from typing import Tuple
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import math
import sys

FLT_EPSILON = sys.float_info.epsilon


# put everything inside a class called pose 




def get_depth(depthframe_, pixel):
    depth_image = bridge.imgmsg_to_cv2(depthframe_, "passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)
    '''
        Given pixel coordinates in an image, the actual image and its depth frame, compute the corresponding depth.
    '''
    heightDEPTH, widthDEPTH = (depth_array.shape[0], depth_array.shape[1])

    x = int(pixel[0])
    y = int(pixel[1])
    def medianCalculation(x, y, width, height, depth_array):
        medianArray = []
        requiredValidValues = 20
        def spiral(medianArray, depth_array, requiredValidValues, startX, startY, endX, endY, width, height):
            if startX <  0 and startY < 0 and endX > width and endY > height:
                return
            # Check first and last row of the square spiral.
            for i in range(startX, endX + 1):
                if i >= width:
                    break
                if startY >= 0 and math.isfinite(depth_array[startY][i]):
                    medianArray.append(depth_array[startY][i])
                if startY != endY and endY < height and math.isfinite(depth_array[endY][i]):
                    medianArray.append(depth_array[endY][i])
                if len(medianArray) > requiredValidValues:
                    return
            # Check first and last column of the square spiral.
            for i in range(startY + 1, endY):
                if i >= height:
                    break
                if startX >= 0 and math.isfinite(depth_array[i][startX]):
                    medianArray.append(depth_array[i][startX])
                if startX != endX and endX < width and math.isfinite(depth_array[i][endX]):
                    medianArray.append(depth_array[i][endX])
                if len(medianArray) > requiredValidValues:
                    return
            # Go to the next outer square spiral of the depth pixel.
            spiral(medianArray, depth_array, requiredValidValues, startX - 1, startY - 1, endX + 1, endY + 1, width, height)
        
        # Check square spirals around the depth pixel till requiredValidValues found.
        spiral(medianArray, depth_array, requiredValidValues, x, y, x, y, width, height)
        if len(medianArray) == 0:
            return float("NaN")

        # Calculate Median
        medianArray.sort()
        return medianArray[len(medianArray) // 2]
    
    # Get the median of the values around the depth pixel to avoid incorrect readings.
    return medianCalculation(x, y, widthDEPTH, heightDEPTH, depth_array)

def deproject_pixel_to_point(cv_image_rgb_info, pixel, depth):
    '''
        Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, 
        compute the corresponding point in 3D space relative to the same camera
        Reference: https://github.com/IntelRealSense/librealsense/blob/e9f05c55f88f6876633bd59fd1cb3848da64b699/src/rs.cpp#L3505
    '''
    def CameraInfoToIntrinsics(cameraInfo):
        intrinsics = {}
        intrinsics["width"] = cameraInfo.width
        intrinsics["height"] = cameraInfo.height
        intrinsics["ppx"] = cameraInfo.K[2]
        intrinsics["ppy"] = cameraInfo.K[5]
        intrinsics["fx"] = cameraInfo.K[0]
        intrinsics["fy"] = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            intrinsics["model"] = "RS2_DISTORTION_BROWN_CONRADY"
        elif cameraInfo.distortion_model == 'equidistant':
            intrinsics["model"] = "RS2_DISTORTION_KANNALA_BRANDT4"
        intrinsics["coeffs"] = [i for i in cameraInfo.D]
        return intrinsics
    
    # Parse ROS CameraInfo msg to intrinsics dictionary.
    intrinsics = CameraInfoToIntrinsics(cv_image_rgb_info)

    if(intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY"): # Cannot deproject from a forward-distorted image
        return

    x = (pixel[0] - intrinsics["ppx"]) / intrinsics["fx"]
    y = (pixel[1] - intrinsics["ppy"]) / intrinsics["fy"]

    xo = x
    yo = y

    if (intrinsics["model"] == "RS2_DISTORTION_INVERSE_BROWN_CONRADY"):
        # need to loop until convergence 
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
            xq = float(x / icdist)
            yq = float(y / icdist)
            delta_x = float(2 * intrinsics["coeffs"][2] * xq * yq + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq))
            delta_y = float(2 * intrinsics["coeffs"][3] * xq * yq + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq))
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
        # need to loop until convergence 
        # 10 iterations determined empirically
        for i in range(10):
            r2 = float(x * x + y * y)
            icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
            delta_x = float(2 * intrinsics["coeffs"][2] * x * y + intrinsics["coeffs"][3] * (r2 + 2 * x * x))
            delta_y = float(2 * intrinsics["coeffs"][3] * x * y + intrinsics["coeffs"][2] * (r2 + 2 * y * y))
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON

        theta = float(rd)
        theta2 = float(rd * rd)
        for i in range(4):
            f = float(theta * (1 + theta2 * (intrinsics["coeffs"][0] + theta2 * (intrinsics["coeffs"][1] + theta2 * (intrinsics["coeffs"][2] + theta2 * intrinsics["coeffs"][3])))) - rd)
            if abs(f) < FLT_EPSILON:
                break
            df = float(1 + theta2 * (3 * intrinsics["coeffs"][0] + theta2 * (5 * intrinsics["coeffs"][1] + theta2 * (7 * intrinsics["coeffs"][2] + 9 * theta2 * intrinsics["coeffs"][3]))))
            theta -= f / df
            theta2 = theta * theta
        r = float(math.tan(theta))
        x *= r / rd
        y *= r / rd

    if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
        rd = float(math.sqrt(x * x + y * y))
        if rd < FLT_EPSILON:
            rd = FLT_EPSILON
        r = (float)(math.tan(intrinsics["coeffs"][0] * rd) / math.atan(2 * math.tan(intrinsics["coeffs"][0] / float(2.0))))
        x *= r / rd
        y *= r / rd

    return (depth * x, depth * y, depth)

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
cv_image_rgb_info = CameraInfo()
def camera_info_callback(data):
    global cv_image_rgb_info
    cv_image_rgb_info = data
cv_image_depth_info = Image()
def camera_depth_info_callback(data):
    global cv_image_depth_info
    cv_image_depth_info = data
# Initializing the ROS node
rospy.init_node('ImageRecever', anonymous=True)

# Subscribing to the ROS topic
imageSub = rospy.Subscriber("/zed2/zed_node/rgb/image_rect_color", Image, image_callback)

#Create subscriber to camera info
camera_info_sub = rospy.Subscriber("/zed2/zed_node/rgb/camera_info", CameraInfo, camera_info_callback)
#Create subscriber to camera info
camera_info_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, camera_depth_info_callback)

# Publishing the pose estimation results
posePub = rospy.Publisher('/pose_estimation', Point, queue_size=10)
image_pose_pub = rospy.Publisher('/image_pose', Image, queue_size=10)
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



            # publishing the pose estimation results in ros image
            # publish image_pose with points in the pose estimation
            # image_pose = Image()
            # image_pose.header.stamp = rospy.Time.now()
            # image_pose.header.frame_id = "zed2_camera_center"
            # image_pose.height = image.shape[0]
            # image_pose.width = image.shape[1]
            # image_pose.encoding = "rgb8"
            # image_pose.is_bigendian = 0
            # image_pose.step = image.shape[1] * 3
            # image_pose.data = image.tobytes()
            # posePub.publish(image_pose)
             
            

            


            # Drawing the pose detection results
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            
            # publish image to a topic called image_pose w
            image_pose = Image()
            image_pose.header.stamp = rospy.Time.now()
            image_pose.header.frame_id = "zed2_camera_center"
            image_pose.height = image.shape[0]
            image_pose.width = image.shape[1]
            image_pose.encoding = "rgb8"
            image_pose.is_bigendian = 0
            image_pose.step = image.shape[1] * 3
            image_pose.data = image.tobytes()
            #convert image to ros image
            image_pose_pub.publish(image_pose)

            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # cv2.imshow('MediaPipe Pose', image)
            # if cv2.waitKey(5) & 0xFF == 27:
            #     break

            if results.pose_landmarks:
                        x = (
                            results.pose_landmarks.landmark[12].x + results.pose_landmarks.landmark[11].x) / 2
                        y = (
                            results.pose_landmarks.landmark[12].y + results.pose_landmarks.landmark[11].y) / 2
                        depth = get_depth(cv_image_depth_info, [x, y])
                        print(de)
                        deproject = deproject_pixel_to_point(cv_image_rgb_info,[x,y],depth)
                        x=deproject[0]
                        y=deproject[1]
                        z=deproject[2]
                        #z = (
                        #    results.pose_landmarks.landmark[12].z + results.pose_landmarks.landmark[11].z) / 2
                        chest = Point()
                        chest.x = x
                        chest.y = y
                        chest.z = z
                        posePub.publish(chest)
        else:
            print("Image not recived")
        sleep(1)