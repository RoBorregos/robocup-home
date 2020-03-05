#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys 
import numpy as np
import cv2


#TODO: pass arguments to node to determine these constants.
# Frequency in hertz to check the camera.
RATE = 10
#Default webcam 
# 0 --- Integrated
# 2 --- Intel B/W
# 3 --- Intel Color
CAMERA = 0


def main():
    # TODO: Change the name of the node and publisher depending what
    # camera this is reading.
    pub = rospy.Publisher("frames", Image, queue_size=RATE)
    rospy.init_node("imageCapture")

    bridge = CvBridge()
    cam = cv2.VideoCapture(0)

    rater = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        _, frame = cam.read()
        
        # ROS image stuff
        if frame is not None:
            frame = np.uint8(frame)
            image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            pub.publish(image_message)

        rater.sleep()
    cam.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

