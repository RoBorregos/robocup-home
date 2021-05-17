import rospy
import random

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

def publisher():
    rospy.loginfo("Starting test publisher")
    pub = rospy.Publisher("/robot_info", RobotStatus, queue_size=10)
    rospy.init_node("test_hri_publisher", anonymous=True)
    rate = rospy.Rate(1/3)

    while not rospy.is_shutdown():
        message = RobotStatus()
        # Set active modules.
        active_robot_modules = []
        for module in robot_modules:
            if bool(random.getrandbits(1)):
                active_robot_modules.append(module)
        message.active_modules = active_robot_modules
        # Set system health.
        message.system_health = [
            # Battery.
            float(85 + int(random.getrandbits(4))),
            # CPU.
            float(random.getrandbits(4)),
            # RAM.
            float(random.getrandbits(4)),
        ]
        # TODO: add values to the rest of the message.
        pub.publish(message)
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException: pass
