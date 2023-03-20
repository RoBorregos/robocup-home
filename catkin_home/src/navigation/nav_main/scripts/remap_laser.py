#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

def laser_scan_callback(msg):
    # Create a new LaserScan message with the same content as the original one
    new_msg = LaserScan()
    new_msg.header = msg.header
    new_msg.angle_min = msg.angle_min
    new_msg.angle_max = msg.angle_max
    new_msg.angle_increment = msg.angle_increment
    new_msg.time_increment = msg.time_increment
    new_msg.scan_time = msg.scan_time
    new_msg.range_min = msg.range_min
    new_msg.range_max = msg.range_max
    new_msg.ranges = msg.ranges
    new_msg.intensities = msg.intensities

    # Update the tf frame name
    new_msg.header.frame_id = "hs/laser_frame"

    # Publish the new message to the "hs/scan" topic
    pub.publish(new_msg)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('laser_scan_republisher')

    # Create a publisher for the modified LaserScan messages
    pub = rospy.Publisher('hs/scan', LaserScan, queue_size=10)

    # Subscribe to the original LaserScan messages
    rospy.Subscriber('scan', LaserScan, laser_scan_callback)

    # Spin until Ctrl+C is pressed
    rospy.spin()
