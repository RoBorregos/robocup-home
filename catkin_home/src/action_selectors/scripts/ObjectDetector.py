#!/usr/bin/env python
'''
This is a node in ROS that takes the published message image and 
calls with certain `RATE` the script object_detection and publishes
the results to `objects_detected`.
'''
import tempfile

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


#TODO: Pass arguments to node to determine these constants.
# Frequency for (aprox) calling the object detection code.
RATE = 3


cv_bridge = None
rater = None

def callback_object_detection(msg):
    rospy.loginfo("Image received")
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    # Save and process the image with the objectdetection script.
    # TODO: Implement the try catch and check the with.
    # TODO: Maybe is desired to actually save the file, then `mkstemp` could be used.
    # Also, maybe try to use `mode=wb` to avoide additional overhead(?).
    with tempfile.NamedTemporaryFile(suffix=".jpg") as temp_img_file:
        # We create a temporal file that can be opened by the other process
        # and that will be deleted when closed.

        # Resize the image to meet the requirements of ObjDetecion.
        # TODO: Check size to see if it will enlarge or shrink and use
        # CV_INTER_AREA. Check https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
        # also check whether changing the ratio is very harmful.
        cv_image = cv2.resize(cv_image, (800, 600), interpolation=cv2.INTER_LINEAR)
        _, img_buffer = cv2.imencode(".jpg", cv_image)

        temp_img_file.write(img_buffer.tostring())
        temp_img_file.flush()

        rospy.loginfo("Tmp image saved (" + temp_img_file.name +"), calling script")

    # Because there is not a spinOnce in python, lets sleep in the
    # callback function to achieve that rate.
    rater.sleep()


def main():
    rospy.init_node("ObjectDetector")
    #pub = rospy.Publisher("objects_detected", Image, queue_size=RATE)
    rospy.Subscriber("frames", Image, callback_object_detection, queue_size=RATE)
    rospy.loginfo("*Node ObjectDetector started*")
    
    global cv_bridge, rater
    cv_bridge = CvBridge()
    rater = rospy.Rate(RATE)
    # TODO: Open ObjectDetection scripts.

    rospy.loginfo("*ObjectDetection module ready to callback*")
    rospy.spin()
    
    rospy.loginfo("*Node finished*")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

