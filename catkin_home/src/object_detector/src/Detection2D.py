#!/usr/bin/env python3
# USAGE
# python3 Detection2d.py

import numpy as np
import argparse
import tensorflow as tf
import cv2
import pathlib
import rospy
import threading
import imutils
import time
from imutils.video import FPS
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Bool
from object_detector.msg import objectDetection, objectDetectionArray
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
import sys
sys.path.append(str(pathlib.Path(__file__).parent) + '/../include')
from vision_utils import *

SOURCES = {
    "VIDEO": str(pathlib.Path(__file__).parent) + "/../resources/test.mp4",
    "CAMERA": 0,
    "ROS_IMG": "/camaras/0",
}

ARGS= {
    "SOURCE": SOURCES["VIDEO"],
    "ROS_INPUT": False,
    "USE_ACTIVE_FLAG": True,
    "DEPTH_ACTIVE": False,
    "DEPTH_INPUT": "/camera/depth/image_raw",
    "CAMERA_INFO": "/camera/depth/camera_info",
    "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/",
    "LABELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/label_map.pbtxt",
    "MIN_SCORE_THRESH": 0.6,
    "VERBOSE": True,
}

class CamaraProcessing:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = []
        self.rgb_image = []
        self.imageInfo = CameraInfo()

        # Load Models
        print("[INFO] Loading models...")
        
        def loadTfModel():
            self.detect_fn = tf.saved_model.load(ARGS["MODELS_PATH"])
            self.category_index = label_map_util.create_category_index_from_labelmap(ARGS["LABELS_PATH"], use_display_name=True)

        loadTfModel()
        print("[INFO] Model Loaded")
        
        self.activeFlag = not ARGS["USE_ACTIVE_FLAG"]
        self.runThread = None
        self.subscriber = None
        self.handleSource()
        self.publisher = rospy.Publisher('detections', objectDetectionArray, queue_size=5)
        self.posePublisher = rospy.Publisher("/test/detectionposes", PoseArray, queue_size=5)
        if ARGS["USE_ACTIVE_FLAG"]:
            rospy.Subscriber('detectionsActive', Bool, self.activeFlagSubscriber)

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

        # Show OpenCV window.
        try:
            self.detections_frame = []
            rate = rospy.Rate(60)
            while not rospy.is_shutdown():
                if ARGS["VERBOSE"] and len(self.detections_frame) != 0:
                    cv2.imshow("Detections", self.detections_frame)
                    cv2.waitKey(1)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cv2.destroyAllWindows()
    
    # Callback for active flag
    def activeFlagSubscriber(self, msg):
        self.activeFlag = msg.data

    # Function to handle either a cv2 image or a ROS image.
    def handleSource(self):
        if ARGS["ROS_INPUT"]:
            self.subscriber = rospy.Subscriber(ARGS["SOURCE"], Image, self.imageRosCallback)
            if ARGS["DEPTH_ACTIVE"]:
                self.subscriberDepth = rospy.Subscriber(ARGS["DEPTH_INPUT"], Image, self.depthImageRosCallback)
                self.subscriberInfo = rospy.Subscriber(ARGS["CAMERA_INFO"], CameraInfo, self.infoImageRosCallback)
        else:
            cThread = threading.Thread(target=self.cameraThread, daemon=True)
            cThread.start()

    # Function to handle a cv2 input.
    def cameraThread(self):
        cap = cv2.VideoCapture(ARGS["SOURCE"])
        frame = []
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                ret, frame = cap.read()
                if not ret:
                    continue
                if len(frame) == 0:
                    continue
                self.imageCallback(frame)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        cap.release()

    # Function to handle a ROS input.
    def imageRosCallback(self, data):
        try:
            self.imageCallback(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
    # Function to handle a ROS depth input.
    def depthImageRosCallback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
    
    # Function to handle ROS camera info input.
    def infoImageRosCallback(self, data):
        self.imageInfo = data
        self.subscriberInfo.unregister()

    # Function to handle Rate Neckbottle, TF object detection model frame rate (<10FPS) against camera input (>30FPS).
    # Process a frame only when the script finishes the process of the previous frame, rejecting frames to keep real-time idea.
    def imageCallback(self, img):
        self.rgb_image = img
        if not self.activeFlag:
            self.detections_frame = img
        elif self.runThread == None or not self.runThread.is_alive():
            self.runThread = threading.Thread(target=self.run, args=(img, ), daemon=True)
            self.runThread.start()

    # Handle FPS calculation.
    def callFps(self):	
        if self.fps != None:
            self.fps.stop()
            print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
            print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
            self.fpsValue = self.fps.fps()

        self.fps = FPS().start()
        
        callFpsThread = threading.Timer(2.0, self.callFps, args=())
        callFpsThread.start()

    # Function to run the detection model.
    def run_inference_on_image(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_np = np.expand_dims(frame, axis=0)
        input_tensor = tf.convert_to_tensor(frame_np, dtype=tf.uint8)

        if ARGS["VERBOSE"]:
            print('Predicting...')

        start_time = time.time()
        detections = self.detect_fn(input_tensor)
        end_time = time.time()
        elapsed_time = end_time - start_time
        
        if ARGS["VERBOSE"]:
            print('Done! Took {} seconds'.format(elapsed_time))

        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        return detections['detection_boxes'], detections['detection_scores'], detections['detection_classes'], detections

    # Handle the detection model input/output.
    def compute_result(self, frame):
        (boxes, scores, classes, detections) = self.run_inference_on_image(frame)
        return self.get_objects(boxes, scores, classes, frame.shape[0], frame.shape[1], frame), detections, frame, self.category_index

    # This function creates the output array of the detected objects with its 2D & 3D coordinates.
    def get_objects(self, boxes, scores, classes, height, width, frame):
        objects = {}
        res = []

        pa = PoseArray()
        pa.header.frame_id = "camera_depth_frame"
        pa.header.stamp = rospy.Time.now()

        for index, value in enumerate(classes):
            if scores[index] > ARGS["MIN_SCORE_THRESH"]:
                if value in objects:
                    # in case it detects more that one of each object, grabs the one with higher score
                    if objects[value]['score'] > scores[index]:
                        continue
                
                point3D = Point()
                point2D = get2DCentroid(boxes[index], self.depth_image)
                
                if ARGS["DEPTH_ACTIVE"] and len(self.depth_image) != 0:
                    depth = get_depth(self.depth_image, point2D)
                    point3D_ = deproject_pixel_to_point(self.imageInfo, point2D, depth)
                    point3D.x = point3D_[0]
                    point3D.y = point3D_[1]
                    point3D.z = point3D_[2]
                    pa.poses.append(Pose(position=point3D))
                objects[value] = {
                    "score": float(scores[index]),
                    "ymin": float(boxes[index][0]),
                    "xmin": float(boxes[index][1]),
                    "ymax": float(boxes[index][2]),
                    "xmax": float(boxes[index][3]),
                    "point3D": point3D
                }
        self.posePublisher.publish(pa)
        
        for label in objects:
            labelText = self.category_index[label]['name']
            detection = objects[label]
            res.append(objectDetection(
                    label = int(label),
                    labelText = str(labelText),
                    score = detection["score"],
                    ymin =  detection["ymin"],
                    xmin =  detection["xmin"],
                    ymax =  detection["ymax"],
                    xmax =  detection["xmax"],
                    point3D = detection["point3D"]
                ))
        return res

    # Main function to run the detection model.
    def run(self, frame):
        frame_processed = imutils.resize(frame, width=500)

        detected_objects, detections, image, category_index = self.compute_result(frame_processed)
        
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            detections['detection_boxes'],
            detections['detection_classes'],
            detections['detection_scores'],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=ARGS["MIN_SCORE_THRESH"],
            agnostic_mode=False)

        self.detections_frame = frame

        self.publisher.publish(objectDetectionArray(detections=detected_objects))
        self.fps.update()

def main():
    rospy.init_node('Vision2D', anonymous=True)
    for key in ARGS:
        ARGS[key] = rospy.get_param('~' + key, ARGS[key])
    CamaraProcessing()

if __name__ == '__main__':
    main()
