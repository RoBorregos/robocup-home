#!/usr/bin/env python3
"""
This script test the trained model and performs object detection on a given image, the script prints
in console a JSON with the objects detected and their info.

E.g. 
>>> python get_objects_and_coordinates_tf2.py -i /home/ricardochapa/Downloads/test_image.jpg -o /home/ricardochapa/Desktop/Roborregos/ -v true

output:
{
    "jumex_cajita": {
        "score": 0.9995954632759094,
        "ymin": 435.3594779968262,
        "xmin": 185.599547624588,
        "ymax": 547.5826263427734,
        "xmax": 248.1843888759613
    },
    "fanta": {
        "score": 0.9212740063667297,
        "ymin": 314.3073320388794,
        "xmin": 328.5956025123596,
        "ymax": 473.17280769348145,
        "xmax": 405.4909944534302
    }
}
"""
import rospy
from std_msgs import *
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from object_detector.msg import ObjectArray, ObjectDetected
import os
import time
# Run tensorflow CPU instead of GPU (because the Jetson Nano runs out of memory when using GPU)
os.environ["CUDA_VISIBLE_DEVICES"]="-1" 
import json

import threading
import argparse
import cv2
import numpy as np
import tensorflow as tf

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Look for object_detection directory based on absolute path to file.
# This allows the scripts to be called from any directory.
base_directory = 'catkin_home'
path_to_file = os.path.abspath(__file__) # get absolute path to current working file
index_of_base_directory = path_to_file.find(base_directory)
WORKING_DIR = path_to_file[0:index_of_base_directory + len(base_directory)] + "/../object_detection"

MODEL_NAME = 'saved_model'
CWD_PATH = os.path.join(WORKING_DIR, 'models', 'model_tf2')
PATH_TO_SAVED_MODEL = os.path.join(CWD_PATH, MODEL_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH,'label_map.pbtxt')
NUM_CLASSES = 4
MIN_SCORE_THRESH = 0.6

VERBOSE = None
category_index = None
thread_reference = None
image_np_with_detections = []
objectArray = []
objects_detected_in_run = []

def load_model():
    """
    This function loads the neccesary files, load the model and returns a function ready
    to make inference.
    """
    # Funcion created by TF2 based on the object detection model
    detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

    # Load the label map.
    global category_index
    category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

    def run_inference_on_image(image):
        # Convert CV2 image from BGR to RGB
        # Comment this line in case the model can take any order of RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np = np.expand_dims(image, axis=0) #np.shape = (1, 800, 600, 3)
        # Load image using OpenCV

        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image_np, dtype=tf.uint8)

        print('Running model...', end='')
        start_time = time.time()

        # Run the model to get the detections dictionary
        detections = detect_fn(input_tensor)

        end_time = time.time()
        elapsed_time = end_time - start_time
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

        #display_image_detection(image, detections)

        return detections['detection_boxes'], detections['detection_scores'], detections['detection_classes'], detections

    return run_inference_on_image

def display_image_detection(image, detections):
    global image_np_with_detections
    """
    This function displays the image with the detections of the objects
    """
    # Visualize the image and detections 
    image_np_with_detections = image.copy()
    vis_util.visualize_boxes_and_labels_on_image_array(
        image_np_with_detections,
        detections['detection_boxes'],
        detections['detection_classes'],
        detections['detection_scores'],
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        min_score_thresh=MIN_SCORE_THRESH,
        agnostic_mode=False)

def get_objects(boxes, scores, classes, height, width):
    """
    This function creates a json of the detected objects with its coordinates.
    """
    objects = {}
    for index,value in enumerate(classes):
        if scores[index] > MIN_SCORE_THRESH:
            if category_index[value]['name'] in objects:
                # in case it detects more that one of each object, grabs the one with higher score
                if objects[category_index[value]['name']]['score'] > scores[index]:
                    continue
            objects[category_index.get(value)['name']] = {
                'score': float(scores[index]),
                'ymin': float(boxes[index][0]*height),
                'xmin': float(boxes[index][1]*width),
                'ymax': float(boxes[index][2]*height),
                'xmax': float(boxes[index][3]*width)
            }
    
    # Return the most compact form of the json.
    return json.dumps(objects, separators=(',',':'), indent=2)

def compute_result(model_call_function, image):
    """
    This function is the model handler for object detection.
    Args:
        rmodel_call_function: TF function to run the objet detection model
        image: the compressed image to detect @type numpy.ndarray
    """

    (boxes, scores, classes, detections) = model_call_function(image)
    
    return get_objects(boxes, scores, classes, image.shape[0], image.shape[1]), detections

def isInObjectArray(object, ObjectArray):
    '''
    This function finds wheter a current object has already being seen or not.
    TODO:
    This function only 
    '''
    for i in range(len(ObjectArray)):
        if (ObjectArray[i].id == str(object)):
            return i
    return -1

def generate_object_detection_msg(frame, detected_objects):
    global objectArray
    global objects_detected_in_run
    detected_objects = json.loads(detected_objects)
    ros_msg = None
    

    # Add new object detection or update the object position if seen again
    for object in detected_objects:
        indexInObjectArray = isInObjectArray(object, objectArray)
        
        if (indexInObjectArray != -1):
            objectArray[indexInObjectArray].in_view = True

            objectArray[indexInObjectArray].object_pose.position.z = 0
            objectArray[indexInObjectArray].object_pose.orientation.x = 0
            objectArray[indexInObjectArray].object_pose.orientation.y = 0
            objectArray[indexInObjectArray].object_pose.orientation.z = 0
            objectArray[indexInObjectArray].object_pose.orientation.w = 0

            image_height, image_width = frame.shape[0], frame.shape[1]
            x_coordinate_in_image = (detected_objects[object]['xmax'] + detected_objects[object]['xmin']) / 2.0
            y_coordinate_in_image = (detected_objects[object]['ymax'] + detected_objects[object]['ymin']) / 2.0

            x_normalized = x_coordinate_in_image / image_width
            y_normalized = y_coordinate_in_image / image_height

            objectArray[indexInObjectArray].object_pose.position.x = x_normalized
            objectArray[indexInObjectArray].object_pose.position.y = y_normalized

        else:
            ros_msg = ObjectDetected()
            ros_msg.id = str(object)
            ros_msg.in_view = True
            ros_msg.object_pose = Pose()

            ros_msg.object_pose.position.z = 0
            ros_msg.object_pose.orientation.x = 0
            ros_msg.object_pose.orientation.y = 0
            ros_msg.object_pose.orientation.z = 0
            ros_msg.object_pose.orientation.w = 0

            image_height, image_width = frame.shape[0], frame.shape[1]
            x_coordinate_in_image = (detected_objects[object]['xmax'] + detected_objects[object]['xmin']) / 2.0
            y_coordinate_in_image = (detected_objects[object]['ymax'] + detected_objects[object]['ymin']) / 2.0

            x_normalized = x_coordinate_in_image / image_width
            y_normalized = y_coordinate_in_image / image_height

            ros_msg.object_pose.position.x = x_normalized
            ros_msg.object_pose.position.y = y_normalized

            objectArray.append(ros_msg)
        
            objects_detected_in_run.append(str(object))

    # update the objects that are not being seen in the frame
    objects_detected_in_frame = list(detected_objects.keys())
    for object in objects_detected_in_run:
        if not (object in objects_detected_in_frame):
            for object_detected_index in range(len(objectArray)):
                if objectArray[object_detected_index].id == str(object):
                    objectArray[object_detected_index].in_view = False
  



def thread_callback(compressedImage):
    """
        This function takes the callback from the image thread to run the 
        object detection model.

        Args:
            compressedImage: image taken from the intelrealsense 
            ros topic camera/color/image_raw/compressed 

    """
    global VERBOSE
    global model_call_function
    global objectArray

    np_arr = np.frombuffer(compressedImage.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    detected_objects, detections = compute_result(model_call_function, frame)

    print(detected_objects)

    if VERBOSE:
       display_image_detection(frame, detections)
       
    objectDetectdMsg = generate_object_detection_msg(frame, detected_objects)

    status_publisher = rospy.Publisher("objects_detected", ObjectArray, queue_size=10)
    status_publisher.publish(objectArray)

def callback(compressedImage):
    """
    Credits to: @JoseCisneros
    The idea of this function is to emulate the behavior of the file 
    object_detection/scripts/get_objects_and_coordinates.py. This file takes the
    input images from the intel camera at a rate of 15 fps generating multiple threads to process the
    detections. 
    
    Since ROS doesn't automatically generate threads for its processes', the rate of detection of this file
    decresases to significally for the detections (although the intel camera updates it's topic at 15 fps, the queue 
    of the callback increases).

    This file generates a thread that takes every update from the intel ros topic /camera/color/image_raw/compressed
    and process every image once the TF object detection model finishes the previouse generated thread.
    """
    global thread_reference
    global image_np_with_detections

    if thread_reference == None or not thread_reference.is_alive():        
        # Update the image shown in the GUI
        if len(image_np_with_detections) != 0:
            cv2.imshow('image_results', image_np_with_detections)
            cv2.waitKey(10)
        
        thread_reference = threading.Thread(target=thread_callback, args=(compressedImage, ), daemon=True)
        thread_reference.start()

def str2bool(v):
    """
    Function to convert from string to boolean
    """
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == '__main__':
    global model_call_function

    # Load the TF model and generate it's function
    model_call_function = load_model()
    
    # image display in GUI boolean variable
    VERBOSE = str2bool(str(rospy.get_param('~VERBOSE', 1)))

    rospy.init_node('get_objects_and_coordinates', anonymous=True)
    print('Node: get_objects_and_coordinates initialized!')
    #image_subscriber = rospy.Subscriber("/camera/color/image_raw/compressed" , CompressedImage, callback)
    image_subscriber = rospy.Subscriber("/camera/color/image_raw/compressed" , CompressedImage, callback)
    rospy.spin()
    cv2.destroyAllWindows()
