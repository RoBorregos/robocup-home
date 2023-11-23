#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def voz_humana():

    rospy.init_node('voz_humana', anonymous=True)
    pub = rospy.Publisher('/tp1', String, queue_size=10)
    rate = rospy.Rate(0.2) # 10hz

    while not rospy.is_shutdown():

        pub.publish("Move to the table, find a person, and introduce yourself")
        rate.sleep()
        

if __name__ == '__main__':
    voz_humana()


