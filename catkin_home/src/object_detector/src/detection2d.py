#!/usr/bin/env python3
# USAGE
# python3 detection2d.py

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
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from object_detector.msg import objectDetection, objectDetectionArray
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

SOURCES = {
    "VIDEO": str(pathlib.Path(__file__).parent) + "/../resources/test.mp4",
    "CAMERA": 0,
    "ROS_IMG": "/camaras/0",
}

ARGS= {
    "SOURCE": SOURCES["VIDEO"],
    "ROS_INPUT": False,
    "USE_ACTIVE_FLAG": True,
    "MODELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/",
    "LABELS_PATH": str(pathlib.Path(__file__).parent) + "/../models/label_map.pbtxt",
    "CONFIDENCE": 0.6,
    "MIN_SCORE_THRESH": 0.6,
    "VERBOSE": True,
}

class CamaraProcessing:

    def __init__(self):
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
        if ARGS["USE_ACTIVE_FLAG"]:
            rospy.Subscriber('detectionsActive', Bool, self.activeFlagSubscriber)

        # Frames per second throughput estimator
        self.fps = None
        callFpsThread = threading.Thread(target=self.callFps, args=(), daemon=True)
        callFpsThread.start()

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
    
    def activeFlagSubscriber(self, msg):
        self.activeFlag = msg.data

    def handleSource(self):
        if ARGS["ROS_INPUT"]:
            self.subscriber = rospy.Subscriber(ARGS["SOURCE"], CompressedImage, self.imageRosCallback)
        else:
            cThread = threading.Thread(target=self.cameraThread, daemon=True)
            cThread.start()

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

    def imageRosCallback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.imageCallback(frame)

    def imageCallback(self, img):
        """
        Rate Neckbottle considering TF object detection model frame rate (<10FPS) against camera input (>30FPS).
        Process a frame only when the script finishes the process of the previous frame, rejecting frames to keep real-time idea.
        """
        if not self.activeFlag:
            self.detections_frame = img
        elif self.runThread == None or not self.runThread.is_alive():
            self.runThread = threading.Thread(target=self.run, args=(img, ), daemon=True)
            self.runThread.start()

    def callFps(self):	
        if self.fps != None:
            self.fps.stop()
            print("[INFO] elapsed time: {:.2f}".format(self.fps.elapsed()))
            print("[INFO] approx. FPS: {:.2f}".format(self.fps.fps()))
            self.fpsValue = self.fps.fps()

        self.fps = FPS().start()
        
        callFpsThread = threading.Timer(2.0, self.callFps, args=())
        callFpsThread.start()

    def run_inference_on_image(self, frame):
        # Convert CV2 frame from BGR to RGB
        # Comment this line in case the model can take any order of RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Expand dimensions since the model expects frames to have shape: [1, None, None, 3]
        frame_np = np.expand_dims(frame, axis=0) #np.shape = (1, 800, 600, 3)
        # Load frame using OpenCV

        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(frame_np, dtype=tf.uint8)

        if ARGS["VERBOSE"]:
            print('Predicting...')
        start_time = time.time()

        # Run the model to get the detections dictionary
        detections = self.detect_fn(input_tensor)

        end_time = time.time()
        elapsed_time = end_time - start_time
        if ARGS["VERBOSE"]:
            print('Done! Took {} seconds'.format(elapsed_time))

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        # Detection_classes should be ints.
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        return detections['detection_boxes'], detections['detection_scores'], detections['detection_classes'], detections

    def compute_result(self, frame):
        (boxes, scores, classes, detections) = self.run_inference_on_image(frame)
        return self.get_objects(boxes, scores, classes, frame.shape[0], frame.shape[1]), detections, frame, self.category_index

    def get_objects(self, boxes, scores, classes, height, width):
        """
        This function creates the output array of the detected objects with its coordinates.
        """
        objects = {}
        res = []
        for index, value in enumerate(classes):
            if scores[index] > ARGS["MIN_SCORE_THRESH"]:
                if value in objects:
                    # in case it detects more that one of each object, grabs the one with higher score
                    if objects[value]['score'] > scores[index]:
                        continue
                objects[value] = {
                    "score": float(scores[index]),
                    "ymin": float(boxes[index][0]),
                    "xmin": float(boxes[index][1]),
                    "ymax": float(boxes[index][2]),
                    "xmax": float(boxes[index][3])
                }
        
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
                ))
        
        return res

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
