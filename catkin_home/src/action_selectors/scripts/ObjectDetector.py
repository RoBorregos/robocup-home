#!/usr/bin/env python
"""
This is a node in ROS that takes the published message image and 
calls with certain `RATE` the script object_detection and publishes
the results to `objects_detected`. To do this, `subprocess` module is
used to write and read to it.

https://stackoverflow.com/a/33886970

This is implemented having a callback that stores the data and another
callback of a timer to actually do the computation, because anyway `rospy`
always callbacks with the published topics, then to achieve the rate was the
only way; also with the logic's delay in topic's callback the msgs were always
super delayed.
Support for the Intel RealSense is also included via the flag --use_rs_camera.

TODO: Try to implement try-except in several parts with the subprocess and
implement timeouts when reading because something could go wrong with the
script. Maybe using `asyncio`, `pexpect`, or others.
"""
import argparse
import json
import tempfile
from os import path
# TODO: Update to subprocess32
import subprocess
import sys

import rospy
import rospkg
from sensor_msgs.msg import Image, CameraInfo
from action_selectors.msg import ObjectsDetected, ObjectDetected
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from DepthSegmentation import RenderImage


argp = argparse.ArgumentParser()
argp.add_argument(
    "--use_rs_camera", 
    help=("Use the Intel RS camera instead of normal `frames` topic. " +
        "This effectively turns on the segmentation algorithms and (in the future) " +
        "enables to publish the results of these."),
    action="store_true",
    default=True)
# Remove the ROS args and start after the name of the program.
args = argp.parse_args(rospy.myargv(argv=sys.argv)[1:])


USE_RS_CAMERA = args.use_rs_camera

# To make others' life easier...
if USE_RS_CAMERA:
    import pyrealsense2 as rs

    from DepthSegmentation import Segmentation

#TODO: Pass arguments to node to determine these constants.
# Frequency for (aprox) calling the object detection code.
RATE = 3

VENV_PATH = '/media/Data/lugol/Desktop/RoBorregos/Robocup-Home/object_detection/bin/env/bin/python3'
# This is relative to "object_detection" dir (`CWD_COMMAND_OBJ_DETECTION`).
PATH_OBJ_DET_SCRIPT = 'scripts/get_objects_and_coordinates_in_terminal.py'

rp = rospkg.RosPack()
# https://answers.ros.org/question/236116/how-do-i-access-files-in-same-directory-as-executable-python-catkin/
obj_det_home_path = path.join(rp.get_path("action_selectors"), "..", "..", "..", "object_detection")

COMMAND_OBJ_DETECTION = path.join(obj_det_home_path, "venv", "env", "bin", "activate")
COMMAND_OBJ_DETECTION += "; "
COMMAND_OBJ_DETECTION += path.join(obj_det_home_path, "venv", "env", "bin", "python3")
COMMAND_OBJ_DETECTION += " "
COMMAND_OBJ_DETECTION += path.join(obj_det_home_path, "scripts", "get_objects_and_coordinates_in_terminal.py")

CWD_COMMAND_OBJ_DETECTION = obj_det_home_path

cv_bridge = None
rater = None
obj_det_process = None

is_new_image = False
actual_msg_img = None
actual_msg_depth = None
rs_intrinsics = None

publisher = None


def callback_topic_img(msg):
    #rospy.loginfo("Image received")
    global actual_msg_img, is_new_image
    actual_msg_img, is_new_image = msg, True

def callback_topic_depth(msg):
    #rospy.loginfo("Depth received")
    # We don't do any kind of sync between depth and color img, only
    # hope they arrive timely and are of the same scene.
    global actual_msg_depth
    actual_msg_depth = msg

def callback_timer_analyze_msg_image(_):
    rospy.loginfo("Entering callback timer")
    RE_SIZE = (800, 600)

    # Quickly save the image locally and reset global img flag in one pass.
    global is_new_image
    if is_new_image:
        msg_img, msg_depth, is_new_image = actual_msg_img, actual_msg_depth, False
        # As there isn't implemented (yet) any sync, we need to ensure
        # depth has arrived at least once in live.
        if USE_RS_CAMERA and msg_depth is None:
            return
    else:
        return

    rospy.loginfo("Analyzing")

    if USE_RS_CAMERA:
        cv_image = cv_bridge.imgmsg_to_cv2(msg_img, desired_encoding="bgr8")
    else:
        cv_image = cv_bridge.imgmsg_to_cv2(msg_img, desired_encoding="passthrough")
    # Resize the image to meet the requirements of ObjDetecion.
    # TODO: Check size to see if it will enlarge or shrink and use
    # CV_INTER_AREA. Check https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
    # also check whether changing the ratio is very harmful.
    cv_image = cv2.resize(cv_image, RE_SIZE, interpolation=cv2.INTER_LINEAR)

    # Save and process the image with the objectdetection script.
    # TODO: Maybe is desired to actually save the file, then `mkstemp` could be used.
    # Also, maybe try to use `mode=wb` to avoide additional overhead(?).
    with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
        # We create a temporal file that can be opened by the other process
        # and that will be deleted when closed.

        _, img_buffer = cv2.imencode(".jpg", cv_image)

        temp_img_file.write(img_buffer.tostring())
        temp_img_file.flush()
        rospy.loginfo("Tmp image saved (" + temp_img_file.name +"), calling script")

        while True:
            if obj_det_process.stdout.readline().decode('utf-8') == '~#waiting input#~\n':
                break
        obj_det_process.stdin.write((temp_img_file.name + '\n').encode('utf-8'))
        obj_det_process.stdin.flush()
        while True:
            if obj_det_process.stdout.readline().decode('utf-8') == '~#result#~\n':
                break
        json_result = obj_det_process.stdout.readline().decode('utf-8')[:-1]

        rospy.loginfo("Results ready")

    #rospy.loginfo("Json result=" + str(json_result))
    detected_objects = json.loads(json_result)

    # Rearrange the data.
    objects = []
    for object_name in detected_objects:
        objects.append(
            ObjectDetected(
                name=str(object_name),
                score=detected_objects[object_name]['score'],
                x_min=detected_objects[object_name]['xmin'],
                y_min=detected_objects[object_name]['ymin'],
                x_max=detected_objects[object_name]['xmax'],
                y_max=detected_objects[object_name]['ymax'],
            ),
        )


    # Segmentation algorithm using depth.
    if USE_RS_CAMERA:
        depth_image = cv_bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
        depth_image = cv2.resize(depth_image, RE_SIZE, interpolation=cv2.INTER_LINEAR)

        for obj in objects[0:2]:
            (angle_x_y, angle_z_y), (center_x, center_y, _) = Segmentation.depth_segment_to_get_center_and_angle_of_object(
                cv_image, depth_image, obj, paint_image=True)
            # Try to adjust (even more, as the center of mass should already account
            # for this) the y_center with the z inclination.
            # center_y += center_y * 0.20 * (angle_z_y / 90.0) if angle_z_y > 0 else center_y * 0.20 * (-angle_z_y / 90.0)
            center_y = center_y * (1 + 1.0 * (90 - angle_z_y) / 90.0) if angle_z_y > 0 else center_y * (1 + 1.0 * (90 + angle_z_y) / 90.0)
            center_y = int(center_y)
            four_limits_obj = Segmentation.depth_trace_limits_of_four_sides_of_object(
                cv_image, depth_image, obj, rs_intrinsics, center_x_y=(center_x, center_y),
                angle_x_y=angle_x_y, paint_image=True)


    show_image_with_boxes(cv_image, objects, wait_millis=50)

    publisher.publish(ObjectsDetected(objects_detected=objects))

def show_image_with_boxes(image, objects, wait_millis=0):
    boxes = []
    category_index = {}
    map_name_id = {}
    scores = []
    classes = []
    for obj in objects:
        if obj.name not in map_name_id:
            index = len(map_name_id) + 1
            map_name_id[obj.name] = index
            category_index[index] = {'id': index, 'name': obj.name}

            classes.append(index)
        else:
            classes.append(map_name_id[obj.name])

        scores.append(obj.score)
        boxes.append([obj.y_min, obj.x_min, obj.y_max, obj.x_max])

    RenderImage.visualize_boxes_and_labels_on_image_array(
        image, np.array(boxes), np.array(classes).astype(np.int32),
        np.array(scores), category_index, use_normalized_coordinates=False,
        line_thickness=8, min_score_thresh=0.6)

    # Display the results image.
    cv2.imshow('image_results', image)
    cv2.waitKey(wait_millis)

def init_obj_det_process():
    '''
    Open the object detection script and leave it ready after start up tasks.
    '''
    try:
        process = subprocess.Popen(
            COMMAND_OBJ_DETECTION,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            # TODO: Check how to send this to null.
            stderr=subprocess.STDOUT,
            bufsize=1,
            shell=True,
            cwd=CWD_COMMAND_OBJ_DETECTION,
            universal_newlines=True,
        )
        if process.returncode != None:
            return None
        rospy.loginfo("PID of opened subprocess=" + str(process.pid))
    except:
        return None
    rospy.loginfo("Process opened successfully")

    while True:
        process_msg = process.stdout.readline().decode('utf-8')
        if(len(process_msg) > 1):
            print "[CHILD PROCESS]", process_msg
        if process_msg == "~#ready#~\n":
            break

    rospy.loginfo("Process is ready")

    return process

def close_obj_det_process():
    rospy.loginfo("Closing process")

    '''
    This is not the best way because the script callback could be interrupted
    while it is stuck in another reading.
    obj_det_process.stdin.write('~#exit#~\n'.encode('utf-8'))
    obj_det_process.stdin.flush()
    # TODO: With while process.poll != None, implement a timeout kill.
    obj_det_process.wait()
    '''
    obj_det_process.kill()
    obj_det_process.wait()

def callback_shutting_down():
    # Because we 'are not able to' inmedietly terminate the node, let's only
    # say that we received it and the node will directly die by itself.
    rospy.loginfo("*Received signal, node will finish (only if it finished setting up)*")


def main():
    rospy.init_node("ObjectDetector")
    rospy.on_shutdown(callback_shutting_down)
    global publisher
    publisher = rospy.Publisher("objects_detected", ObjectsDetected, queue_size=RATE)
    rospy.loginfo("*Node ObjectDetector started*")
    
    global cv_bridge, rater, obj_det_process
    cv_bridge = CvBridge()
    rater = rospy.Rate(RATE)
    obj_det_process = init_obj_det_process()
    if obj_det_process == None:
        rospy.loginfo("*ERROR: Process could not be opened, closing.*")
        exit()

    seconds = int(1/RATE)
    timer_analyze = rospy.Timer(rospy.Duration(seconds, int(1000000000*(1.0/RATE - seconds))), callback_timer_analyze_msg_image)
    # `queue_size=1` to always get the last one.
    if USE_RS_CAMERA:
        # Let's wait to get the intrinsics of the depth.
        # TODO: Obviously this isn't the best way, as if the config of the
        # camera changes or just that this blocks the start.
        rospy.loginfo("Waiting to receive camera info topic...")
        depth_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)
        global rs_intrinsics
        # IntelRealSense/realsense-ros: realsense2_camera/src/base_realsense_node.cpp::BaseRealSenseNode::updateStreamCalibData()
        rs_intrinsics = rs.intrinsics()
        rs_intrinsics.height = depth_info.height
        rs_intrinsics.width = depth_info.width
        rs_intrinsics.ppx = depth_info.K[2]
        rs_intrinsics.ppy = depth_info.K[5]
        rs_intrinsics.fx = depth_info.K[0]
        rs_intrinsics.fy = depth_info.K[4]
        # rs_intrinsics.coeffs = []
        for i in range(len(depth_info.D)): rs_intrinsics.coeffs.append(depth_info.D[i])
        # This is hardcoded as (at this point) isn't correctly set. Check updateStreamCalibData() 
        rs_intrinsics.model = rs.distortion.inverse_brown_conrady

        # TODO: Changing to use `image_transport` instead of the raw, could be good idea.
        rospy.Subscriber(
            "/camera/color/image_raw", Image, callback_topic_img, queue_size=1)
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, callback_topic_depth, queue_size=1)
    else:
        rospy.Subscriber("frames", Image, callback_topic_img, queue_size=1)
    rospy.loginfo("*ObjectDetection module ready to callback*")
    rospy.spin()

    rospy.loginfo("*Finishing node*")
    close_obj_det_process()
    timer_analyze.shutdown()
    cv2.destroyAllWindows()
    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

