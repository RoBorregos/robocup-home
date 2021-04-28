"""
This code is used test the model on a single image using Google Colab notebooks.
"""

import os
import cv2
import numpy as np
import tensorflow as tf
import sys

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

#from google.colab.patches import cv2_imshow

# sys.path.append("..")

MODEL_NAME = 'saved_model'
CWD_PATH = './models/model_tf2'
PATH_TO_SAVED_MODEL = os.path.join(CWD_PATH, MODEL_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH,'label_map.pbtxt')
PATH_TO_IMAGE = './test_images/test_image_tf2.jpg'
NUM_CLASSES = 4
MIN_SCORE_THRESH = 0.6


# Load the label map.
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

# Load image using OpenCV
image = cv2.imread(PATH_TO_IMAGE)

# Convert CV2 image from BGR to RGB
# Comment this line in case the model can take any order of RGB
imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_expanded = np.expand_dims(imageRGB, axis=0)

# Define the inout image as a tensor
input_tensor = tf.convert_to_tensor(image_expanded, dtype=tf.uint8)

# Funcion created by TF2 based on the object detection model
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

# Run the model to get the detections dictionary
detections = detect_fn(input_tensor)

# All outputs are batches tensors.
# Convert to numpy arrays, and take index [0] to remove the batch dimension.
# We're only interested in the first num_detections.
num_detections = int(detections.pop('num_detections'))
detections = {key: value[0, :num_detections].numpy()
            for key, value in detections.items()}
detections['num_detections'] = num_detections

# Detection_classes should be ints.
detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

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
