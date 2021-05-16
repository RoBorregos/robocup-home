#!/usr/bin/env python3
import sys 
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

# FPS.
RATE = int(rospy.get_param('~RATE', 15))
# Default webcam.
CAMERA = int(rospy.get_param('~CAMERAID', 0))
# Show Images.
VERBOSE = False

def main():
    rospy.init_node("ImageCapture" + str(CAMERA))
    image_publisher = rospy.Publisher("camaras/" + str(CAMERA) + "/" , CompressedImage, queue_size = RATE)
    rospy.loginfo("*Node " + "ImageCapture-" + str(CAMERA) + " started*")

    cam = cv2.VideoCapture(CAMERA)

    rospy_rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        _, frame = cam.read()
        
        # Image Display.
        if VERBOSE:
            cv2.imshow('cv_img', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise Exception('Stop executing')

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        
        # Publish new image.
        image_publisher.publish(msg)

        rospy_rate.sleep()
    
    cam.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass