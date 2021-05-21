#!/usr/bin/env python3
import sys 
import cv2
import rospy
import numpy as np
from devices.srv import GetCameraIndex
from sensor_msgs.msg import CompressedImage

# Show Images.
VERBOSE = True

def returnCameraIndex(CAMERA):
    index = 0
    count = 0
    while index < 15:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            cap.release()
            if count == CAMERA:
                return index
            count += 1
        index += 1
    return -1

def main():
    rospy.init_node("ImageCapture")
    # FPS.
    RATE = int(rospy.get_param('~RATE', 15))
    # Default webcam.
    CAMERA = int(rospy.get_param('~CAMERAID', 0))
    rospy.loginfo("CAMERA" + str(CAMERA))
    image_publisher = rospy.Publisher("camaras/" + str(CAMERA) + "/" , CompressedImage, queue_size = RATE)
    rospy.loginfo("*Node " + "ImageCapture-" + str(CAMERA) + " started*")

    rospy.wait_for_service('get_camera_index')
    get_camera_index = rospy.ServiceProxy('get_camera_index', GetCameraIndex)
    getCamIndex = get_camera_index(CAMERA)
    if getCamIndex.camera_index == -1:
        raise Exception('Connection with Camera' + str(CAMERA) + ' unsuccessful')
    cam = cv2.VideoCapture(getCamIndex.camera_index)

    rospy_rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        check, frame = cam.read()
        
        if not check:
            continue
        
        # Image Display.
        if VERBOSE:
            cv2.imshow('cv_img' + str(CAMERA), frame)
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