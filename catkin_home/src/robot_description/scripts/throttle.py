#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

class ThrottleFilteringPointsNode:
    def __init__(self):
        self.pc_sub = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.pc_callback)
        self.pc_pub = rospy.Publisher("/throttle_filtering_points/filtered_points", PointCloud2, queue_size=10)
        self.rate = rospy.Rate(2)  # 2Hz
        self.last_msg = None

    def pc_callback(self, msg):
        self.last_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.last_msg is not None:
                self.last_msg.header.stamp = rospy.Time.now()
                self.pc_pub.publish(self.last_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('throttle_filtering_points')
    node = ThrottleFilteringPointsNode()
    node.run()
