#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def voz_humana():

    rospy.init_node('voz_humana', anonymous=True)
    pub = rospy.Publisher('/tp1', String, queue_size=10)
    rate = rospy.Rate(0.2)

    while not rospy.is_shutdown():
        # Request activation call
        activation_call = input()
        pub.publish(activation_call)
        
        rate.sleep()
        

if __name__ == '__main__':
    voz_humana()


