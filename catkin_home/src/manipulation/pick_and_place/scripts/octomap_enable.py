#!/usr/bin/env python3

from pick_and_place.srv import EnableOctomap, EnableOctomapResponse
from sensor_msgs.msg import PointCloud2
import rospy
import time

FORWARD_PCL = True
PUBLISHER_PCL = False
SUBSCRIBER_PCL = False

def points_callback(data):
  global FORWARD_PCL
  global PUBLISHER_PCL
  global SUBSCRIBER_PCL
  if FORWARD_PCL and PUBLISHER_PCL:
    PUBLISHER_PCL.publish(data)

def handle_enable_octomap(req):
    global FORWARD_PCL
    global PUBLISHER_PCL
    global SUBSCRIBER_PCL
    FORWARD_PCL = req.enable
    return EnableOctomapResponse(True)

def enable_octomap_server():
    global FORWARD_PCL
    global PUBLISHER_PCL
    global SUBSCRIBER_PCL
    rospy.init_node('enable_octomap_server')
    PUBLISHER_PCL = rospy.Publisher('/hsrb/octomap/points', PointCloud2, queue_size=10)
    SUBSCRIBER_PCL = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/points", PointCloud2, points_callback, queue_size=1)
    rospy.Service('enable_octomap', EnableOctomap, handle_enable_octomap)
    print("Ready to enable/disable octomap.")
    rospy.spin()

if __name__ == "__main__":
    enable_octomap_server()
