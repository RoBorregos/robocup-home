#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys 
import numpy as np
import cv2
#TODO pass arguments to node to determine the webcam to be used.
def main():
    #declares the publisher with numpy_msg type of
    #imports external message
    pub = rospy.Publisher("image",Image,queue_size=10)
    rospy.init_node("imageCapture",anonymous=True)
    bridge = CvBridge()
    if(sys.argv):
        print("Argument")
    #Default webcam 
    # 0 --- Integrated
    # 2 --- Intel B/W
    # 3 --- Intel Color
    cam = cv2.VideoCapture(0)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        _, frame = cam.read()
        # ROS image stuff
        if frame is not None:
            frame = np.uint8(frame)
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(image_message)
    cam.release()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass