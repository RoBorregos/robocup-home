#!/usr/bin/env python
import rospy
from action_selectors.msg import RawInput

def hear():
    pub = rospy.Publisher('RawInput', RawInput, queue_size=10)
    rospy.init_node('hear', anonymous=True) ##Hear is initialized
    rate = rospy.Rate(1) # 10hz
    isWoman=False
    while not rospy.is_shutdown():
        ##Cardozo code here --------------
        msg = RawInput()
        msg.isWoman =isWoman
        msg.inputText = "RandomTExt %s" % rospy.get_time() 
        ##------------------
        rospy.loginfo(msg)
        pub.publish(msg)
        isWoman=not isWoman
        rate.sleep()

if __name__ == '__main__':
    try:
        hear()
    except rospy.ROSInterruptException:
        pass
