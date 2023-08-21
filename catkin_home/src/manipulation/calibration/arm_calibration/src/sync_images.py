#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

class CameraSyncNode:
		def __init__(self):
			rospy.init_node('camera_sync_node', anonymous=True)
			self.bridge = CvBridge()

			rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback)
			rospy.Subscriber('camera_image', Image, self.camera_image_callback)
			
			self.synced_camera_info_pub = rospy.Publisher('synced_camera_info', CameraInfo, queue_size=10)
			self.synced_image_pub = rospy.Publisher('synced_camera_image', Image, queue_size=10)

			self.new_camera_info = False
			self.new_image = False
			self.camera_info = None
			self.image = None

			self.run()
			
		def camera_info_callback(self, msg):
			self.camera_info = msg
			self.new_camera_info = True
		
		def camera_image_callback(self, msg):
			self.image = msg
			self.new_image = True
		
		def sync_images(self):
			if self.new_camera_info and self.new_image:
				self.camera_info.header.stamp = self.image.header.stamp
				self.synced_camera_info_pub.publish(self.camera_info)
				self.synced_image_pub.publish(self.image)
				self.new_camera_info = False
				self.new_image = False
		
		def run(self):
			rate = rospy.Rate(10)
			while not rospy.is_shutdown():
				self.sync_images()
				rate.sleep()
			

		

if __name__ == '__main__':
		try:
				camera_sync_node = CameraSyncNode()
		except rospy.ROSInterruptException:
				pass
