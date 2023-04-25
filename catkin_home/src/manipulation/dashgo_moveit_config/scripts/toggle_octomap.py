#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

class OctomapService:
    def __init__(self):
        self.state = True
        self.sub = rospy.Subscriber('/zed2/zed_node/point_cloud/ds_cloud_registered', PointCloud2, self.cloud_callback)
        self.pub = rospy.Publisher('/octomap/points', PointCloud2, queue_size=10)
        self.service = rospy.Service('/toggle_octomap', SetBool, self.toggle_octomap)
        self.state_pub = rospy.Publisher('/octomap/state', Bool, queue_size=10)
        
    def cloud_callback(self, data):
        if self.state:
            self.pub.publish(data)

    def toggle_octomap(self, req):
        self.state = req.data
        self.state_pub.publish(self.state)
        return SetBoolResponse(success=True, message='Octomap state changed to {}'.format(self.state))

if __name__ == '__main__':
    rospy.init_node('octomap_service_node')
    OctomapService()
    rospy.spin()
