#!/usr/bin/env python3
import sys 
import cv2
import rospy
import numpy as np
from devices.srv import GetCameraIndex

cameraIndexes = []
def get_camera_index(msg):
    global cameraIndexes
    print(cameraIndexes, msg)
    if msg.id < len(cameraIndexes):
        return cameraIndexes[msg.id]
    return -1

def returnCameraIndexes():
    global cameraIndexes
    index = 0
    count = 0
    while index < 15:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            cap.release()
            cameraIndexes.append(index)
            count += 1
        index += 1

def main():
    rospy.init_node("CameraIndex")
    returnCameraIndexes()
    service = rospy.Service('get_camera_index', GetCameraIndex, get_camera_index)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass