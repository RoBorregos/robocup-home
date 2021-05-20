#!/usr/bin/env python3
'''
This script creates the node `SomeoneToTalk` that taking an input video 
from a webCamara, publishes if there is a person in front of the camara.

Use to start a conversation with a person.
'''
import rospy
import scipy
import cv2
import sys
import os
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

class SomeoneToTalk(object):
    dirname = os.path.dirname(__file__)
    cascPath = os.path.join(dirname, './FaceDetection/haarcascade_frontalface_default.xml')
    lastStatus = False
    countSameStatus = 0
    
    def __init__(self, CAMERA):
        self.image_subscriber = rospy.Subscriber("camaras/" + str(CAMERA) + "/" , CompressedImage, self.callback)
        self.status_publisher = rospy.Publisher("someoneToTalkStatus", Bool, queue_size=10)
        self.faceCascade = cv2.CascadeClassifier(self.cascPath)
    
    def callback(self, compressedImage):
        np_arr = np.frombuffer(compressedImage.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        if len(faces) > 0:
            if self.lastStatus == True:
                self.countSameStatus += 1
            else:
                self.lastStatus = True
                self.countSameStatus = 0
        else:
            if self.lastStatus == False:
                self.countSameStatus += 1
            else:
                self.lastStatus = False
                self.countSameStatus = 0
        
        if self.countSameStatus >= 10:
            self.status_publisher.publish(Bool(self.lastStatus))

def main():
    rospy.init_node('SomeoneToTalk', anonymous=True)
    CAMERA = int(rospy.get_param('~CAMERAID', 0))
    rospy.loginfo("*Starting SomeoneToTalk Node*")
    SomeoneToTalk(CAMERA)
    rospy.spin()

if __name__ == '__main__':
    main()
