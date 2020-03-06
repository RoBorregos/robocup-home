"""
This script, done to interface with catkin_home/action_selectors/ObjectDetector.py, is to test the trained 
model and performs object detection on given images, then it prints a JSON with the objects detected and their info.
The mark points are marked as ~#label#~

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
# Run tensorflow CPU instead of GPU (because the Jetson Nano runs out of memory when using GPU)
os.environ["CUDA_VISIBLE_DEVICES"]="-1" 
import json

import cv2
import numpy as np
import tensorflow as tf

from utils import label_map_util
from utils import visualization_utils as vis_util


MODEL_NAME = 'inference_graph'
CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')
PATH_TO_LABELS = os.path.join(CWD_PATH,'training','labelmap.pbtxt')
NUM_CLASSES = 4
MIN_SCORE_THRESH = 0.6


category_index = None


def load_model():
    """
    This function loads the neccesary files, load the model and returns a function ready
    to make inference.
    """
    # Load the label map.
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    global category_index
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

    def run_inference_on_image(image):
        # Load image using OpenCV
        image_expanded = np.expand_dims(image, axis=0)

        # Perform the actual detection by running the model with the image as input.
        return sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})

    return run_inference_on_image

def get_objects_json(boxes, scores, classes, height, width):
    """
    This function creates a json of the detected objects with its coordinates.
    """
    objects = {}

    for index,value in enumerate(classes[0]):
        if scores[0,index] > MIN_SCORE_THRESH:
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
    
    # Return the most compact form of the json.
    return json.dumps(objects, separators=(',',':'))
    
def compute_result(run_inference_on_image, path_to_img):
    image = cv2.imread(path_to_img)

    (boxes, scores, classes, _) = run_inference_on_image(image)
    return get_objects_json(boxes, scores, classes, image.shape[0], image.shape[1])


if __name__ == '__main__':
    print("*Script started*")
    run_inference_on_image = load_model()
    print("*Model loaded*")

    print("~#ready#~")

    while True:
        print("~#waiting input#~")
        img_path = input()
        if img_path == "~#exit#~":
            break

        json_result = compute_result(run_inference_on_image, img_path)
        print("~#result#~")
        print(json_result)

    print("*Script ended*")

