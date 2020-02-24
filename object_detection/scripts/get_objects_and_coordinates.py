"""
This script test the trained model and performs object detection on a given image, the script prints
in console a JSON with the objects detected and their info.

E.g. 
>>> python get_objects_and_coordinates.py -d /home/roborregos/Documents/Robocup-Home/object_detection/test_images/image4.jpg -o /home/roborregos/Documents/ -i true

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

import os
os.environ["CUDA_VISIBLE_DEVICES"]="-1" 
import argparse
import cv2
import numpy as np
import tensorflow as tf
import sys
import json

from utils import label_map_util
from utils import visualization_utils as vis_util

#from google.colab.patches import cv2_imshow

def run_model(image_directory):
    """
    This function loads the neccesary files, runs the model and gets the detection boxes, scores and classes.
    """
    global boxes, scores, classes, num, category_index, image

    MODEL_NAME = 'inference_graph'
    CWD_PATH = os.getcwd()
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')
    PATH_TO_LABELS = os.path.join(CWD_PATH,'training','labelmap.pbtxt')
    PATH_TO_IMAGE = image_directory
    NUM_CLASSES = 4

    # Load the label map.
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.Session(graph=detection_graph)

    # Define input and output tensors (i.e. data) for the object detection classifier
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Load image using OpenCV
    image = cv2.imread(PATH_TO_IMAGE)
    image_expanded = np.expand_dims(image, axis=0)

    # Perform the actual detection by running the model with the image as input
    (boxes, scores, classes, num) = sess.run(
        [detection_boxes, detection_scores, detection_classes, num_detections],
        feed_dict={image_tensor: image_expanded})

def get_objects():
    """
    This function creates a dict of the detected objects with its coordinates.
    """
    height = image.shape[0]
    width = image.shape[1]
    objects = {}

    for index,value in enumerate(classes[0]):
        if scores[0,index] > 0.6:
            if category_index.get(value)['name'] in objects:
                # in case it detects more that one of each object, grabs the one with higher score
                if objects[category_index.get(value)['name']]['score'] > scores[0,index]:
                    continue
            objects[category_index.get(value)['name']] = {
                'score': float(scores[0,index]),
                'ymin': float(boxes[0][index][0]*height),
                'xmin': float(boxes[0][index][1]*width),
                'ymax': float(boxes[0][index][2]*height),
                'xmax': float(boxes[0][index][3]*width)
            }
    
    return objects

def display_image_results():
    """
    This function draws the results of the detection (aka 'visulaize the results')
    """
    vis_util.visualize_boxes_and_labels_on_image_array(
        image,
        np.squeeze(boxes),
        np.squeeze(classes).astype(np.int32),
        np.squeeze(scores),
        category_index,
        use_normalized_coordinates=True,
        line_thickness=8,
        min_score_thresh=0.6)

    # Display the results image.
    cv2.imshow('image_results', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Get objects and coordinates")
    parser.add_argument('-d', '--directory', type=str, required=True, help='Directory containing the image')
    parser.add_argument('-o', '--output_directory', type=str, required=True, help='Output directory where the JSON containing the results will be posted')
    parser.add_argument('-i', '--image_view', type=str, required=True, help='Display the image results (true/false)')
    args = parser.parse_args()

    run_model(args.directory)
    detected_objects = get_objects()

    # Write JSON results to a txt file
    with open(os.path.join(args.output_directory,'detected_objects.txt'),'w') as outfile:
        json.dump(detected_objects, outfile)

    print(json.dumps(detected_objects, indent=4))
    
    if (args.image_view == 'true'):
        display_image_results()
