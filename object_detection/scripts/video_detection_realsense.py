import numpy as np
import argparse
import tensorflow as tf
import cv2
import pathlib
import tempfile
import os

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
#import pyrealsense2 as rs

import get_objects_and_coordinates
from get_objects_and_coordinates import load_model, compute_result

# patch tf1 into `utils.ops`
utils_ops.tf = tf.compat.v1

# Patch the location of gfile
tf.gfile = tf.io.gfile



category_index = None

def run_inference_for_single_image(model, image):
    image = np.asarray(image)
    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis,...]
    
    # Run inference
    output_dict = model(input_tensor)

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(output_dict.pop('num_detections'))
    output_dict = {key: value[0, :num_detections].numpy()
                   for key, value in output_dict.items()}
    output_dict['num_detections'] = num_detections

    # detection_classes should be ints.
    output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
   
    # Handle models with masks:
    if 'detection_masks' in output_dict:
        # Reframe the the bbox mask to the image size.
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                                    output_dict['detection_masks'], output_dict['detection_boxes'],
                                    image.shape[0], image.shape[1])      
        detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5, tf.uint8)
        output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
    
    return output_dict


def run_inferenceintel(run_inferance_on_image, pipeline):
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
            # We create a temporal file that can be opened by the other process
            # and that will be deleted when closed.

            _, img_buffer = cv2.imencode(".jpg", color_image)
        
            temp_img_file.write(img_buffer.tostring())
            temp_img_file.flush()

            json_result = compute_result(run_inferance_on_image, temp_img_file.name)
            print(json_result)
        
        '''
        run the model from get objects and coordinates and save the result on a usb
        '''
        '''# Actual detection.
        output_dict = run_inference_for_single_image(model, color_image)
        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
            color_image,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8)
        cv2.imshow('object_detection', cv2.resize(color_image, (800, 600)))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break'''

def run_inference(run_inferance_on_image, cap):
    while True:
        ret, image_np = cap.read()
        
        with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
            # We create a temporal file that can be opened by the other process
            # and that will be deleted when closed.

            _, img_buffer = cv2.imencode(".jpg", image_np)
        
            temp_img_file.write(img_buffer.tostring())
            temp_img_file.flush()

            json_result = compute_result(run_inferance_on_image, temp_img_file.name)
            print(json_result)

        '''# Actual detection.
        output_dict = run_inference_for_single_image(model, image_np)
        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8)
        cv2.imshow('object_detection', cv2.resize(image_np, (800, 600)))
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break'''

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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Detect objects inside webcam videostream')
    parser.add_argument('-c', '--camera', type=bool, required=False, help='Camera choose')

    args = parser.parse_args()

    #detection_model = load_model(args.model)
    #category_index = label_map_util.create_category_index_from_labelmap(args.labelmap, use_display_name=True)
    intel = args.camera

    run_inferance_on_image = load_model()

    if intel==True:
        pipeline = create_intelrs_pipeline()
        run_inferenceintel(run_inferance_on_image, pipeline)
    else:
        cap = cv2.VideoCapture(0)
        run_inference(run_inferance_on_image, cap)
