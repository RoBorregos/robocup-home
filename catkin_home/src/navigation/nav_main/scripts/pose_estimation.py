#!/usr/bin/env python3
import mediapipe as mp
from time import sleep
from typing import Tuple
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped, PointStamped, PoseWithCovarianceStamped
import math
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseArray, Pose
import tf2_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import tf2_geometry_msgs  # Import the tf2_geometry_msgs library
FLT_EPSILON = sys.float_info.epsilon


# put everything inside a class called pose 




class DetectorHumano:
    def __init__(self):
        self.bridge = CvBridge()
        self.posePublisher = rospy.Publisher("/pose_estimation", Point, queue_size=5)
        self.posePublisher2 = rospy.Publisher("/person_pose", PointStamped, queue_size=5)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        self.pubPoint = rospy.Publisher('transformed_point', PointStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/robot_pose', Pose, self.odom_callback)
        self.activate_follow =  rospy.Subscriber('/follow_person',Bool,self.callback_follow)
        self.follow_person=False
        self.mask  = None
        self.cv_image = None
        self.camera_info = None
        rospy.loginfo("Subscribed to image")
        # Calling the pose solution from MediaPipe
        self.mp_pose = mp.solutions.pose
        # Calling the solution for image drawing from MediaPipe
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.image_pose_pub = rospy.Publisher('/image_pose', Image, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        # Define NED reference frame origin (change these values to match your use case)
        self.ned_origin = np.array([0.0, 0.0, 0.0])
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_follow = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.x, self.y =0,0
    def callback_follow(self, msg):
        self.follow_person = msg.data
    # Function to handle a ROS depth input.
    def odom_callback(self, msg):
        # Extract the position and orientation from the odometry message
        self.ned_origin = np.array([msg.position.x, msg.position.y, msg.position.z])
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle a ROS camera info input.
    def infoImageRosCallback(self, data):
        self.camera_info = data
        self.subscriberInfo.unregister()


    def callback(self, data):
        self.cv_image = data#self.bridge.imgmsg_to_cv2(data,desired_encoding="rgb8")
        self.get_objects()



    def get_objects(self):
        point3D = Point()
        if self.cv_image is not None and self.camera_info is not None :
            with self.mp_pose.Pose(
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5) as pose:
                    # Converting the ROS image to OpenCV
                    #image = self.cv_image
                    image = self.bridge.imgmsg_to_cv2(self.cv_image, "rgb8")

                    # Detecting the pose with the image
                    image.flags.writeable = False
                    results = pose.process(image)

                    # Drawing the pose detection results
                    image.flags.writeable = True
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    self.mp_drawing.draw_landmarks(
                        image,
                        results.pose_landmarks,
                        self.mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
                         # publish image to a topic called image_pose w
                    if results.pose_landmarks:
                        self.x = (
                                results.pose_landmarks.landmark[12].x + results.pose_landmarks.landmark[11].x) / 2
                        self.y = (
                            results.pose_landmarks.landmark[12].y + results.pose_landmarks.landmark[11].y) / 2
                        self.x = int(self.x*image.shape[1])
                        self.y = int(self.y*image.shape[0])
                        point2D  = [self.x, self.y]
                        if len(self.depth_image) != 0:
                            depth = self.get_depth(self.depth_image, point2D)
                            point3D_ = self.deproject_pixel_to_point(self.camera_info, point2D, depth)
                            point3D.x = point3D_[2]
                            point3D.y = point3D_[0]
                            point3D.z = point3D_[1]
                            point_x = PointStamped()
                            point_x.header.frame_id = 'zed2_camera_center'
                            point_x.point.x = point3D.x 
                            point_x.point.y = -point3D.y 
                            point_x.point.z = 0
                            # Use the TF buffer to lokup the transform from 'zed2_base_link' to 'map'
                            #transform = self.tf_buffer.lookup_transform('map',"zed2_camera_center", rospy.Time(0), rospy.Duration(1.0))
                            # Transform the point to the 'map' frame
                            #point_map = tf2_geometry_msgs.do_transform_point(point_x, transform)
                            try:
                                self.tf_buffer.can_transform('map', 'zed2_camera_center', rospy.Time())
                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                rospy.logerr('Transformation from body_frame to map_frame is not available')
                                rospy.signal_shutdown('Transformation not available')

                            # Convert the object pose from the body frame to the map frame
                            try:
                                object_pose_map = self.tf_buffer.transform(point_x, 'map', rospy.Duration(1.0))
                                self.posePublisher2.publish(object_pose_map)
                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                rospy.logerr('Failed to transform object pose from body_frame to map_frame')
                                rospy.signal_shutdown('Transformation failed')
                            # point_x.point.x = point3D.x 
                            # point_x.point.y = point3D.y 
                            # point_x.point.z = point3D.z
                            # Publish point
                            if self.follow_person:
                                pose = PoseStamped()
                                pose.header.stamp = rospy.Time.now()
                                pose.header.frame_id = 'map'
                                pose.pose.position.x = object_pose_map.point.x
                                pose.pose.position.y = object_pose_map.point.y
                                pose.pose.position.z = 0
                                pose.pose.orientation.x = 0
                                pose.pose.orientation.y = 0
                                pose.pose.orientation.z = 0
                                pose.pose.orientation.w = 1
                                #Publish nav goal
                                self.pub_follow.publish(pose)
              
                    image = cv2.circle(image, (self.x, self.y), 20, (255,0,0), 2)
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
                    self.image_pose_pub.publish(image_pose)

        
    def get_depth(self,depth_image, pixel):
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

    def deproject_pixel_to_point(self,cv_image_rgb_info, pixel, depth):
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
   
if __name__ == '__main__':
    rospy.logwarn("Starting depth")
    rospy.init_node('detector_humano', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    humano = DetectorHumano()
    try :
        while not rospy.is_shutdown():
            humano.get_objects()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard interrupt detected, stopping listener")
