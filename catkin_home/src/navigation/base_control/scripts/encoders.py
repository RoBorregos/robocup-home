#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from base_control.msg import StampedEncoders

def move():
    # Starts a new node
    rospy.init_node('encoders_simulation', anonymous=True)
    rate = rospy.Rate(4) # ROS Rate at 4Hz
    front_publisher = rospy.Publisher('/base_control/front/encoders', StampedEncoders, queue_size=10)
    back_publisher = rospy.Publisher('/base_control/back/encoders', StampedEncoders, queue_size=10)
    
    rospy.loginfo("Node Initiated")
    
    #Initialize Twist Message
    enc_msg = StampedEncoders()
    enc_msg.encoders.left_wheel = 10000
    enc_msg.encoders.right_wheel = 10000
    enc_msg.encoders.time_delta = 0.005

    forward  = 10000
    backward = -10000
    stop     = 0 
    #Direction:      0     , 45   , 90   , 135   , 180   , 225   , 270   , 315     Left   Right   
    encFrontL  =    [forward,forward,forward,stop,backward,backward,backward,stop,backward,forward]
    encBackL   =    [backward,stop,forward,forward,forward,stop,backward,backward,backward,forward]
    encFrontR  =    [backward,stop,forward,forward,forward,stop,backward,backward,forward,backward]
    encBackR   =    [forward,forward,forward,stop,backward,backward,backward,stop,forward,backward]
    
    t0 = rospy.Time.now().to_sec()
    t1 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
    
        if rospy.Time.now().to_sec() - t0 > 5:
            t0 = rospy.Time.now().to_sec()
            
            front_publisher.publish(enc_msg)
            back_publisher.publish(enc_msg)
        
        if rospy.Time.now().to_sec() - t1 > 5:
            t1 = rospy.Time.now().to_sec()
            enc_msg.encoders.left_wheel *= -1
            enc_msg.encoders.right_wheel *= -1

        #Publish Rate
        rate.sleep()
        

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass