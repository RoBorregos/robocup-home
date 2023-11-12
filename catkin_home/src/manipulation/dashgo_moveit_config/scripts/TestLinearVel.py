#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

rospy.init_node('twist_publisher_node')

# Create a publisher for the /smoother_cmd_vel topic
pub = rospy.Publisher('/smoother_cmd_vel', Twist, queue_size=10)

# Create a Twist message with a linear x velocity of -0.1
twist_msg = Twist()
twist_msg.linear.x = 0.1

# Publish the message for 5 seconds
start_time = time.time()
while (time.time() - start_time) < 5:
    pub.publish(twist_msg)
    rospy.sleep(1/75.0)

# Stop the robot by publishing a message with zero velocities
twist_msg.linear.x = 0.0
pub.publish(twist_msg)

# Exit the node
rospy.signal_shutdown('Finished publishing twist commands')