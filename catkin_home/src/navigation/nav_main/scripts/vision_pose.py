#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from vision.msg import objectDetection, objectDetectionArray
import pathlib
from geometry_msgs.msg import Point, PoseArray, Pose

# import sys
# sys.setrecursionlimit(10**6)

import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

class DetectorAruco:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/aruco_out', Image, queue_size=10)
        self.pubData = rospy.Publisher('detect_markers', objectDetectionArray, queue_size=5)
        self.pubmarker = rospy.Publisher('markers', Int32, queue_size=10)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        self.sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.callback)
        self.subscriberDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthImageRosCallback)
        self.subscriberInfo = rospy.Subscriber("/zed2/zed_node/depth/camera_info", CameraInfo, self.infoImageRosCallback)
        
        self.pubmask = rospy.Publisher('/mask_aruco', Image, queue_size=10)
        self.mask  = None
        self.cv_image = np.array([])
        rospy.loginfo("Subscribed to image")
        self.main()


        # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)


    # Function to handle a ROS camera info input.
    def infoImageRosCallback(self, data):
        self.camera_info = data
        self.subscriberInfo.unregister()


    def detectar_arucos(self):
        frame = self.cv_image
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_with_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.cv_image = frame_with_markers

        bb = []
        detections = []
        tempo = []
        if ids is not None:
            if corners:
                for i, marker_corners in enumerate(corners):
                    print(ids[i])
                    corner = corners[0][0]
                    xmayor = np.amax(corner[:, 0])
                    ymayor = np.amax(corner[:, 1])
                    xmenor = np.amin(corner[:, 0])
                    ymenor = np.amin(corner[:, 1])

                    #print(f"Xmayor: {xmayor:.2f}, Xmenor: {xmenor:.2f}, Ymayor: {ymayor:.2f}, Ymenor: {ymenor:.2f}")    
                    tempo =  ymenor, xmenor, ymayor, xmayor
                    bb.append(tempo)
                    detections.append(ids[i])
                    #ids[i][j] Es el id del aruco
                    #corners Es la bounding box del aruco
            self.get_objects(bb, detections)


        self.pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8"))


    def callback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        self.detectar_arucos()


    def get_objects(self, boxes, detections):
        res = []

        pa = PoseArray()
        pa.header.frame_id = "camera_depth_frame"
        pa.header.stamp = rospy.Time.now()
        for index in range(len(boxes)):
            if True:
                point3D = Point()
                rospy.logwarn("---------------------------")


                rospy.logwarn("pose")
                point2D  = get2DCentroid(boxes[index])
                rospy.logwarn(point2D)
                # Dummy point2d

                if len(self.depth_image) != 0:
                    depth = get_depth(self.depth_image, point2D)
                    point3D_ = deproject_pixel_to_point(self.camera_info, point2D, depth)
                    point3D.x = point3D_[0]
                    point3D.y = point3D_[1]
                    point3D.z = point3D_[2]
                    pa.poses.append(Pose(position=point3D))
                    res.append(
                    objectDetection(

                        label = int(index), # 1
                        labelText = str(detections[index]), # "H"
                        #score = float(0.0),
                        ymin = float(boxes[index][0]),
                        xmin = float(boxes[index][1]),
                        ymax = float(boxes[index][2]),
                        xmax = float(boxes[index][3]),
                        point3D = point3D
                    )
                )
            self.posePublisher.publish(pa)

        self.pubData.publish(objectDetectionArray(detections=res))                

        
    
    def main(self):
        rospy.logwarn("Starting listener")
        rospy.init_node('detector_arucos', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try :
            while not rospy.is_shutdown():
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.logwarn("Keyboard interrupt detected, stopping listener")


if __name__ == '__main__':
    DetectorAruco()