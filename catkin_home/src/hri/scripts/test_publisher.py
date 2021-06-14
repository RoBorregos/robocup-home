import cv2
import rospy
import random
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from hri.msg import RobotStatus

robot_modules = [
    "speech",
    "nlu",
    "navigation",
    "object_recognition",
    "person_recognition",
    "mechanism_control",
    "main_engine"
]

vid = cv2.VideoCapture(0)
bridge = CvBridge()

def video_publisher():
    rospy.loginfo("Starting video publisher")
    image_pub = rospy.Publisher("/robot_video_feed", Image, queue_size=10)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            # Publish video feed.
            _, img = vid.read()
            image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            image_pub.publish(image_message)
            rate.sleep()
    except:
        pass

def status_publisher():
    rospy.loginfo("Starting status publisher")
    info_pub = rospy.Publisher("/robot_info", RobotStatus, queue_size=10)
    rate = rospy.Rate(1/3)
    try:
        while not rospy.is_shutdown():
            # Publish robot status.
            message = RobotStatus()
            # Set active modules.
            active_robot_modules = []
            for module in robot_modules:
                if bool(random.getrandbits(1)):
                    active_robot_modules.append(module)
            message.ActiveModules = active_robot_modules
            # Set system health.
            message.SystemHealth = [
                # Battery.
                float(85 + int(random.getrandbits(4))),
                # CPU.
                float(random.getrandbits(4)),
                # RAM.
                float(random.getrandbits(4)),
            ]
            # TODO: add values to the rest of the message.
            info_pub.publish(message)
            rate.sleep()
    except:
        pass
            

if __name__ == "__main__":
    rospy.init_node("test_hri_publisher", anonymous=True)
    threading.Thread(target=video_publisher, daemon=True).start()
    threading.Thread(target=status_publisher, daemon=True).start()
    rospy.spin()
