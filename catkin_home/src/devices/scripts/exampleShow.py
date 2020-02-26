#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys 
import numpy as np
import cv2



def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv2.imshow("frames",cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return
    print(cv_image)
   
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('exampleShow', anonymous=True)

    rospy.Subscriber("frames", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except:
        cv2.destroyAllWindows()
if __name__ == '__main__':
    listener()
