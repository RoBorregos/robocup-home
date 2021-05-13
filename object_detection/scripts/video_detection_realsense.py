import numpy as np
import argparse
import tensorflow as tf
import cv2
import pathlib
import tempfile
import os
import json

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
import pyrealsense2 as rs

import get_objects_and_coordinates
from get_objects_and_coordinates import load_model, compute_result, get_objects

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile

category_index = None

def run_inferenceintel(run_inferance_on_image, pipeline):
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
            # We create a temporal file that can be opened by the other process
            # and that will be deleted when closed.

            _, img_buffer = cv2.imencode(".jpg", color_image)
        
            temp_img_file.write(img_buffer.tostring())
            temp_img_file.flush()

            detected_objects, detections, image, category_index = compute_result(run_inferance_on_image, temp_img_file.name, False)
            print(detected_objects)

            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            detections['detection_boxes'],
            detections['detection_classes'],
            detections['detection_scores'],
            category_index,
            use_normalized_coordinates=True,
            max_boxes_to_draw=200,
            min_score_thresh=0.6,
            agnostic_mode=False)
            cv2.imshow('object_detection', cv2.resize(image, (800, 600)))
            if cv2.waitKey(25) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                break

def run_inference(run_inferance_on_image, cap):
    while True:
        ret, image_np = cap.read()
        
        with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
            # We create a temporal file that can be opened by the other process
            # and that will be deleted when closed.

            _, img_buffer = cv2.imencode(".jpg", image_np)
        
            temp_img_file.write(img_buffer.tostring())
            temp_img_file.flush()

            detected_objects, detections, image, category_index = compute_result(run_inferance_on_image, temp_img_file.name, False)
            print(detected_objects)

        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
        image,
        detections['detection_boxes'],
        detections['detection_classes'],
        detections['detection_scores'],
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        min_score_thresh=0.6,
        agnostic_mode=False)
        cv2.imshow('object_detection', cv2.resize(image, (800, 600)))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

def create_intelrs_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    ##
    depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    ##

    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline

def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Detect objects inside webcam videostream')
    parser.add_argument('-m', '--model', type=str, required=False, help='Model Path')
    parser.add_argument('-l', '--labelmap', type=str, required=False, help='Path to Labelmap')
    parser.add_argument('-c', '--camera', type=str2bool, nargs='?' ,required=True, const=True, default=False ,help='Camera choose')

    args = parser.parse_args()

    #detection_model = load_model(args.model)
    #category_index = label_map_util.create_category_index_from_labelmap(args.labelmap, use_display_name=True)
    intel = args.camera

    run_inferance_on_image = load_model()

    #print(args)

    if intel==True:
        pipeline = create_intelrs_pipeline()
        run_inferenceintel(run_inferance_on_image, pipeline)
    else:
        cap = cv2.VideoCapture(0)
        run_inference(run_inferance_on_image, cap)
