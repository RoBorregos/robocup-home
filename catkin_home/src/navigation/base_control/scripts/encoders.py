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
    enc_msgFront = StampedEncoders()
    enc_msgBack  = StampedEncoders()
    enc_msgFront.encoders.time_delta = 0.005
    enc_msgBack.encoders.time_delta = 0.005

    forward  = 10000
    backward = -10000
    stop     = 0 
   
    #Direction:      0       ,180     ,90     ,270     ,45     ,225     ,135    ,315     ,Left    ,Right   
    encFrontL  =    [forward ,backward,forward,backward,forward,backward,stop   ,stop    ,backward,forward ]
    encBackL   =    [backward,forward ,forward,backward,stop   ,stop    ,forward,backward,backward,forward ]
    encFrontR  =    [backward,forward ,forward,backward,stop   ,stop    ,forward,backward,forward ,backward]
    encBackR   =    [forward ,backward,forward,backward,forward,backward,stop   ,stop    ,forward ,backward]
    
    t0 = rospy.Time.now().to_sec()
    t1 = rospy.Time.now().to_sec()
    indexDirection = 0
    while not rospy.is_shutdown():
    
        if rospy.Time.now().to_sec() - t0 > 5:
            t0 = rospy.Time.now().to_sec()
            
            front_publisher.publish(enc_msgFront)
            back_publisher.publish(enc_msgBack)
        
        if rospy.Time.now().to_sec() - t1 > 5:
            t1 = rospy.Time.now().to_sec()
            indexDirection = indexDirection + 1
            if indexDirection > 9:
                indexDirection = 0
            
        enc_msgFront.encoders.left_wheel  = encFrontL[indexDirection]
        enc_msgBack.encoders.left_wheel   = encBackL[indexDirection]
        enc_msgFront.encoders.right_wheel = encFrontR[indexDirection]
        enc_msgBack.encoders.right_wheel  = encBackR[indexDirection]

        #Publish Rate
        rate.sleep()
        

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass