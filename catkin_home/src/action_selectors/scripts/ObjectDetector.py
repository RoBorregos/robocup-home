#!/usr/bin/env python
'''
This is a node in ROS that takes the published message image and 
calls with certain `RATE` the script object_detection and publishes
the results to `objects_detected`.
'''
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys 
import numpy as np
import cv2


#TODO: pass arguments to node to determine these constants.
# Frequency for (aprox) calling the object detection code.
RATE = 3


cv_bridge = None
rater = None

def callback_object_detection(msg):
    rospy.loginfo("Image received")
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    # TODO: Save and process the image with the objectdetection script.
    
    # Because there is not a spinOnce in python, lets sleep in the
    # callback function to achieve that rate.
    rater.sleep()


def main():
    rospy.init_node("ObjectDetector")
    #pub = rospy.Publisher("objects_detected", Image, queue_size=RATE)
    rospy.Subscriber("frames", Image, callback_object_detection, queue_size=RATE)
    rospy.loginfo("*Node ObjectDetector started*")
    
    global cv_bridge, rater
    cv_bridge = CvBridge()
    rater = rospy.Rate(RATE)
    # TODO: Open ObjectDetection scripts.

    rospy.loginfo("*ObjectDetection module ready to callback*")
    rospy.spin()
    
    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

