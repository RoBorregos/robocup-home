#!/usr/bin/env python3
"""
This script has an implementation to allow the base to move while the arm is moving.
1-. Fix Nav Odom Frame so it doesn't consider moveit base movement.
https://github.com/RoBorregos/home-simulation/blob/22cdabc11514fd063fbee8cac8a49bc61a0e41cf/catkin_ws/src/main_engine/scripts/fix_tf.py
"""

import rospy
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from rospy.timer import Rate
from rospy import Time
import roslib
from tf import transformations as tfs
import tf
import numpy

class MoveitBaseController:
    def __init__(self):
      # Initialize ROS node
      rospy.init_node('moveit_base_controller')
      
      self.robot = moveit_commander.RobotCommander()

      # Set up TF listener and broadcaster
      self.listener = tf.TransformListener()
      self.broadcaster = tf.TransformBroadcaster()

      # Set up publishers for cmd_vel and joint_states topics
      self.publish_rate = rospy.Rate(10)
      self.run()

    def run(self):
      while not rospy.is_shutdown():
        # Fix Transforms
        try:
          (trans,rot) = self.listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          (trans,rot) = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1])
        try:
          transform = tfs.concatenate_matrices(tfs.translation_matrix(trans), tfs.quaternion_matrix(rot))
          (trans_diff,rot_diff) = self.listener.lookupTransform('base_link', 'internal_odom', rospy.Time(0))
          transform_diff = tfs.concatenate_matrices(tfs.translation_matrix(trans_diff), tfs.quaternion_matrix(rot_diff))
          transform = numpy.matmul(transform, transform_diff)
          self.broadcaster.sendTransform(tfs.translation_from_matrix(transform),
            tfs.quaternion_from_matrix(transform),
                          rospy.Time.now(),
                          "internal_odom","odom")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue

        # Sleep for the publishing rate
        self.publish_rate.sleep()

MoveitBaseController()
