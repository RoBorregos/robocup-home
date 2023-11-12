#!/usr/bin/env python3
from time import sleep
from typing import Tuple
import cv2
import mediapipe as mp
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from humanAnalyzer.msg import pose_positions
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Int32
import math
import sys
FLT_EPSILON = sys.float_info.epsilon

# indexToName = ["nose", "leftEyeInner", "leftEye", "leftEyeOuter", "rightEyeInner", "rightEye", "rightEyeOuter", "leftEar", "rightEar", "mouthLeft", "mouthRight", "leftShoulder", "rightShoulder", "leftElbow", "rightElbow", "leftWrist", "rightWrist", "leftPinky", "rightPinky", "leftIndex", "rightIndex", "leftThumb", "rightThumb", "leftHip", "rightHip", "leftKnee", "rightKnee", "leftAnkle", "rightAnkle", "leftHeel", "rightHeel", "leftFootIndex", "rightFootIndex"]

PublisherPoints = [
    {"name": "shoulderLeft", "index": 11}, {"name": "shoulderRight", "index": 12}, {"name": "elbowLeft", "index": 13}, {"name": "elbowRight", "index": 14}, {"name": "wristLeft", "index": 15},  {"name": "wristRight", "index": 16}, {"name": "pinkyLeft", "index": 17}, {"name": "pinkyRight", "index": 18}, {"name": "indexLeft", "index": 19}, {"name": "indexRight", "index": 20}, {"name": "thumbLeft", "index": 21},{"name": "thumbRight", "index": 22}, {"name": "hipLeft", "index": 23}, {"name": "hipRight", "index": 24}, # {"name": "chest", "index": 33},
]


class PoseDetector:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.imageReceved = None

        self.bridge = CvBridge()

        rospy.init_node('PoseDetector')

        self.imageSub = rospy.Subscriber(
            '/zed2_up/zed_up_node/rgb/image_rect_color/compressed', CompressedImage, self.image_callback, queue_size=10)

        self.imageInfo = CameraInfo()
        self.subscriberInfo = rospy.Subscriber("/zed2_up/zed_up_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)

        self.depth_image = []
        self.subscriberDepth = rospy.Subscriber("/zed2_up/zed_up_node/depth/depth_registered", Image, self.depthImageRosCallback)

        self.image_pose_pub = rospy.Publisher(
            '/mediapipe/output', Image, queue_size=10)

        self.posePub = rospy.Publisher(
            "pose", pose_positions, queue_size=10)

        # Publish Chest Point
        self.chestPub = rospy.Publisher(
            "chest", PointStamped, queue_size=10)

        self.pointingPub = rospy.Publisher(
            "pointing", Int32, queue_size=10)

    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def infoImageRosCallback(self, data):
        self.imageInfo = data
        self.subscriberInfo.unregister()

    def image_callback(self, data):
        self.imageReceved = data

    def run(self):
        with self.mp_pose.Pose(
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as pose:
            while not rospy.is_shutdown():
                if self.imageReceved is not None:
                    image = self.bridge.compressed_imgmsg_to_cv2(
                        self.imageReceved, "rgb8")
                    image.flags.writeable = False
                    results = pose.process(image)
                    
                    if results.pose_landmarks:
                        x = (
                            results.pose_landmarks.landmark[12].x + results.pose_landmarks.landmark[11].x) / 2
                        y = (
                            results.pose_landmarks.landmark[12].y + results.pose_landmarks.landmark[11].y) / 2
                        z = (
                            results.pose_landmarks.landmark[12].z + results.pose_landmarks.landmark[11].z) / 2
                        x = int(x*image.shape[1])
                        y = int(y*image.shape[0])
                        point2D = [x, y]
                        if len(self.depth_image) != 0:
                            depth = self.get_depth(self.depth_image, point2D) # IN MM
                            depth = depth/1000 # IN M
                            point3D_ = self.deproject_pixel_to_point(
                                self.imageInfo, point2D, depth)
                            x = point3D_[0]
                            y = point3D_[1]
                            z = point3D_[2]
                        
                        chestPoint = PointStamped()
                        chestPoint.header.frame_id = self.imageReceved.header.frame_id
                        chestPoint.point.x = x
                        chestPoint.point.y = y
                        chestPoint.point.z = z
                        self.chestPub.publish(chestPoint)
                        
                        posePublish = pose_positions()
                        
                        for(i, landmark) in enumerate(results.pose_landmarks.landmark[11:25]):
                            point = Point()
                            initName = PublisherPoints[i]["name"]
                            point.x = landmark.x
                            point.y = landmark.y
                            point.z = landmark.z
                            posePublish.__setattr__(initName, point)
                        
                        point = Point()
                        point.x = x
                        point.y = y
                        point.z = z

                        posePublish.chest = point
                        self.posePub.publish(posePublish)

                        # Find Pointing
                        pointing = Int32()

                        left_shoulder = results.pose_landmarks.landmark[11]
                        right_shoulder = results.pose_landmarks.landmark[12]
                        left_index = results.pose_landmarks.landmark[19]
                        right_index = results.pose_landmarks.landmark[20]
                        h = image.shape[0]
                        w = image.shape[1]

                        m = 0.05

                        right_out = right_index.x*w < right_shoulder.x*w - m*w 
                        left_out = left_index.x*w > left_shoulder.x*w + m*w

                        if right_out and left_out :
                            pointing.data = 0 # Undefined
                        elif right_out:
                            pointing.data = 1 # Right
                        elif left_out:
                            pointing.data = -1 # Left
                        else:
                            pointing.data = 0
                        if pointing.data == 0:
                            color = (0, 0, 255)
                        elif pointing.data == 1:
                            color = (0, 255, 0)
                        else:
                            color = (255, 0, 0)
                        
                        self.pointingPub.publish(pointing)

                        # VERBOSE
                        image = cv2.circle(image, (int(left_shoulder.x*w), int(left_shoulder.y*h)), 5, (0, 0, 255), -1)
                        image = cv2.putText(image, "left_shoulder", (int(left_shoulder.x*w), int(left_shoulder.y*h)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        image = cv2.circle(image, (int(right_shoulder.x*w), int(right_shoulder.y*h)), 5, (0, 0, 255), -1)
                        image = cv2.putText(image, "right_shoulder", (int(right_shoulder.x*w), int(right_shoulder.y*h)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        image = cv2.circle(image, (int(left_index.x*w), int(left_index.y*h)), 5, (0, 0, 255), -1)
                        image = cv2.putText(image, "left_index", (int(left_index.x*w), int(left_index.y*h)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        image = cv2.circle(image, (int(right_index.x*w), int(right_index.y*h)), 5, (0, 0, 255), -1)
                        image = cv2.putText(image, "right_index", (int(right_index.x*w), int(right_index.y*h)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        image = cv2.rectangle(image, (int(left_shoulder.x*w + m*w), 0), (int(right_shoulder.x*w - m*w), h), color, 3)


                    image_pose = Image()
                    image_pose.header.stamp = rospy.Time.now()
                    image_pose.header.frame_id = self.imageReceved.header.frame_id
                    image_pose.height = image.shape[0]
                    image_pose.width = image.shape[1]
                    image_pose.encoding = "rgb8"
                    image_pose.is_bigendian = 0
                    image_pose.step = image.shape[1] * 3
                    image_pose.data = image.tobytes()
                    # convert image to ros image
                    self.image_pose_pub.publish(image_pose)

                    # cv2.imshow('MediaPipe Pose', image)


                    # # draw rect from left_shoulder.x, left_shoulder.y to right_shoulder.x, image.height 
                    # h = image.shape[0]
                    # if pointing.data == 0:
                    #     color = (0, 0, 255)
                    # elif pointing.data == 1:
                    #     color = (255, 0, 0)
                    # else:
                    #     color = (0, 255, 0)
                    # # cv2.rectangle(image, (left_shoulder.x, left_shoulder.y), (right_shoulder.x, h), color, 2)
                    # # cv2.imshow('MediaPipe Pose', image)
                    if cv2.waitKey(1) & 0xFF == 27:
                        break
                else:
                    pass
                rospy.Rate(30).sleep()

    def get_depth(self, depth_image, pixel):
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
                if startX < 0 and startY < 0 and endX > width and endY > height:
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
                spiral(medianArray, depth_array, requiredValidValues,
                       startX - 1, startY - 1, endX + 1, endY + 1, width, height)

            # Check square spirals around the depth pixel till requiredValidValues found.
            spiral(medianArray, depth_array, requiredValidValues,
                   x, y, x, y, width, height)
            if len(medianArray) == 0:
                return float("NaN")

            # Calculate Median
            medianArray.sort()
            return medianArray[len(medianArray) // 2]

        # Get the median of the values around the depth pixel to avoid incorrect readings.
        return medianCalculation(x, y, widthDEPTH, heightDEPTH, depth_array)

    def deproject_pixel_to_point(self, cv_image_rgb_info, pixel, depth):
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

        # Cannot deproject from a forward-distorted image
        if(intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY"):
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
                icdist = float(
                    1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
                xq = float(x / icdist)
                yq = float(y / icdist)
                delta_x = float(
                    2 * intrinsics["coeffs"][2] * xq * yq + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq))
                delta_y = float(
                    2 * intrinsics["coeffs"][3] * xq * yq + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq))
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
            # need to loop until convergence
            # 10 iterations determined empirically
            for i in range(10):
                r2 = float(x * x + y * y)
                icdist = float(
                    1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
                delta_x = float(
                    2 * intrinsics["coeffs"][2] * x * y + intrinsics["coeffs"][3] * (r2 + 2 * x * x))
                delta_y = float(
                    2 * intrinsics["coeffs"][3] * x * y + intrinsics["coeffs"][2] * (r2 + 2 * y * y))
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON

            theta = float(rd)
            theta2 = float(rd * rd)
            for i in range(4):
                f = float(theta * (1 + theta2 * (intrinsics["coeffs"][0] + theta2 * (
                    intrinsics["coeffs"][1] + theta2 * (intrinsics["coeffs"][2] + theta2 * intrinsics["coeffs"][3])))) - rd)
                if abs(f) < FLT_EPSILON:
                    break
                df = float(1 + theta2 * (3 * intrinsics["coeffs"][0] + theta2 * (5 * intrinsics["coeffs"][1] + theta2 * (
                    7 * intrinsics["coeffs"][2] + 9 * theta2 * intrinsics["coeffs"][3]))))
                theta -= f / df
                theta2 = theta * theta
            r = float(math.tan(theta))
            x *= r / rd
            y *= r / rd

        if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON
            r = (float)(math.tan(intrinsics["coeffs"][0] * rd) / math.atan(
                2 * math.tan(intrinsics["coeffs"][0] / float(2.0))))
            x *= r / rd
            y *= r / rd

        return (depth * x, depth * y, depth)

PoseDetector().run()