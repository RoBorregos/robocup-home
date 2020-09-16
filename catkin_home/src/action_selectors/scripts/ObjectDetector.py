#!/usr/bin/env python
'''
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

TODO: Try to implement try-except in several parts with the subprocess and
implement timeouts when reading because something could go wrong with the
script. Maybe using `asyncio`, `pexpect`, or others.
'''
import argparse
import json
import tempfile
from os import path
# TODO: Update to subprocess32
import subprocess
import sys

import rospy
import rospkg
from sensor_msgs.msg import Image
from action_selectors.msg import ObjectsDetected, ObjectDetected
from cv_bridge import CvBridge, CvBridgeError
import cv2


argp = argparse.ArgumentParser()
argp.add_argument(
    "--use_rs_camera", 
    help=("Use the Intel RS camera instead of normal `frames` topic. " +
        "This effectively turns on the segmentation algorithms and (in the future) " +
        "enables to publish the results of these."),
    action="store_true",
    default=False)
# Remove the ROS args and start after the name of the program.
args = argp.parse_args(rospy.myargv(argv=sys.argv)[1:])


USE_RS_CAMERA = args.use_rs_camera

#TODO: Pass arguments to node to determine these constants.
# Frequency for (aprox) calling the object detection code.
RATE = 3

VENV_PATH = '/home/roborregos/object_detection/bin/python3'
# This is relative to "object_detection" dir (`CWD_COMMAND_OBJ_DETECTION`).
PATH_OBJ_DET_SCRIPT = 'scripts/get_objects_and_coordinates_in_terminal.py'

rp = rospkg.RosPack()
# https://answers.ros.org/question/236116/how-do-i-access-files-in-same-directory-as-executable-python-catkin/
obj_det_home_path = path.join(rp.get_path("action_selectors"), "..", "..", "..", "object_detection")
COMMAND_OBJ_DETECTION = [
    # To activate and use the virtual env, use its python executable.
    VENV_PATH,
    # To receive instantaneously outputs and inputs, make it unbuffered.
    '-u',
    PATH_OBJ_DET_SCRIPT,
]
CWD_COMMAND_OBJ_DETECTION = obj_det_home_path
# TODO: This could be relative.
ENV_VARS_COMMAND = {
    "PYTHONPATH": path.join(obj_det_home_path, "models", "research") + ":" + path.join(obj_det_home_path, "models", "research", "object_detection"),
}


cv_bridge = None
rater = None
obj_det_process = None

is_new_image = False
actual_msg_img = None
actual_msg_depth = None

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
    #rospy.loginfo("Entering callback timer")
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

    # Segmentation algorithm using depth.
    if USE_RS_CAMERA:
        depth_image = cv_bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
        depth_image = cv2.resize(depth_image, RE_SIZE, interpolation=cv2.INTER_LINEAR)

        # TODO(dc): Add the actual algorithm call here.

    # Prepare the data and publish it.
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
    publisher.publish(ObjectsDetected(objects_detected=objects))

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
            env=ENV_VARS_COMMAND,
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
        if process.stdout.readline().decode('utf-8') == "~#ready#~\n":
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
    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

