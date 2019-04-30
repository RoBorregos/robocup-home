#!/usr/bin/env python
import rospy
from audio_common_msgs.msg import AudioData
import numpy as np
import matplotlib.pyplot as plt


def callback(data):
    print("LISTENED")
    print("DATA DATA", type(data))
    print(data.data)
    arr = [(ord(i)) for i in data.data]
    print(arr)
    nA=np.array(arr)
    print("PLAYED")
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('playb', anonymous=True)

    rospy.Subscriber("UsefulAudio", AudioData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()