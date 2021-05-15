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
from std_msgs.msg import Bool

dirname = os.path.dirname(__file__)
cascPath = os.path.join(dirname, './FaceDetection/haarcascade_frontalface_default.xml')

def main():
    rospy.init_node('SomeoneToTalk', anonymous=True)
    rospy.loginfo("*Starting SomeoneToTalk Node*")
    status_publisher = rospy.Publisher('someoneToTalkStatus', Bool, queue_size=10)

    faceCascade = cv2.CascadeClassifier(cascPath)

    video_capture = cv2.VideoCapture(0)
    
    r = rospy.Rate(10) # 10FPS
    lastStatus = False
    countSameStatus = 0
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = video_capture.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        if len(faces) > 0:
            if lastStatus == True:
                countSameStatus += 1
            else:
                lastStatus = True
                countSameStatus = 0
        else:
            if lastStatus == False:
                countSameStatus += 1
            else:
                lastStatus = False
                countSameStatus = 0
        
        if countSameStatus >= 10:
            status_publisher.publish(Bool(lastStatus))

        r.sleep()

    # When everything is done, release the capture
    video_capture.release()
    
    rospy.spin()

if __name__ == '__main__':
    main()
