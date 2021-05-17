#!/usr/bin/env python3

import roslaunch
import os
import rospy

def main():
    # Start ObjectDetector.py and ImageCapturer.py
    packages = ['action_selectors','devices']
    executables = ['ObjectDetector.py','ImageCapturer.py']
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    for index,exe in enumerate(executables):
        print("Trying to start")
        node = roslaunch.core.Node(packages[index], executables[index])
        process = launch.launch(node)
        rospy.loginfo("started: " + executables[index])
    
if __name__=="__main__":
    main()