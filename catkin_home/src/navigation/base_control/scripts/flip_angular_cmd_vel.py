#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def callback(data):
    # Flip the sign of the z component of the Twist message
    data.angular.z = -(data.angular.z)
    pub.publish(data)

rospy.init_node('flip_angular_cmd_vel')
sub = rospy.Subscriber('robot_cmd_vel', Twist, callback)
pub = rospy.Publisher('flipped_cmd_vel', Twist, queue_size=10)
rospy.spin()
