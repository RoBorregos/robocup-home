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

import os
import time
# Run tensorflow CPU instead of GPU (because the Jetson Nano runs out of memory when using GPU)
os.environ["CUDA_VISIBLE_DEVICES"]="-1" 
import sys
import json

import argparse
import cv2
import numpy as np
import tensorflow as tf

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

MODEL_NAME = 'saved_model'
CWD_PATH = './models/model_tf2'
PATH_TO_SAVED_MODEL = os.path.join(CWD_PATH, MODEL_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH,'label_map.pbtxt')
NUM_CLASSES = 4
MIN_SCORE_THRESH = 0.6

category_index = None

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

        print('Loading model...', end='')
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
    return objects

def display_image_detection(image, detections):
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

    # Display the results image.
    cv2.imshow('image_results', image_np_with_detections)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def compute_result(run_inference_on_image, path_to_img, image_view):
    global category_index
    image = cv2.imread(path_to_img)

    (boxes, scores, classes, detections) = run_inference_on_image(image)
    
    return get_objects(boxes, scores, classes, image.shape[0], image.shape[1]), detections, image, category_index

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Get objects and coordinates")
    parser.add_argument('-i', '--input_directory', type=str, required=True, help='Input directory containing the image to analize')
    parser.add_argument('-o', '--output_directory', type=str, required=True, help='Output directory to write the JSON results')
    parser.add_argument('-v', '--image_view', type=str, required=True, help='Display the image results (true/false)')
    args = parser.parse_args()

    run_inferance_on_image = load_model()
    detected_objects, detections, image, _ = compute_result(run_inferance_on_image, args.input_directory, args.image_view)
    
    # Write JSON results to a txt file
    with open(os.path.join(args.output_directory,'detected_objects.txt'),'w') as outfile:
        json.dump(detected_objects, outfile, indent=2)

    if args.image_view == "true":
        display_image_detection(image, detections)
