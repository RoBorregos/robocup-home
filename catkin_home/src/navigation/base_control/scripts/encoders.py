#!/usr/bin/env python
'''
    Script that simulate physical robot action of sending encoders ticks.
'''
import rospy
from base_control.msg import StampedEncoders

def run_encoders_simulation():
    '''
        Main function that simulate the action of sending encoders ticks.
    '''
    # Starts a new node.
    rospy.init_node('encoders_simulation', anonymous=True)
    rate = rospy.Rate(4) # ROS Rate at 4Hz.
    front_publisher = rospy.Publisher('/base_control/front/encoders',StampedEncoders,queue_size=10)
    back_publisher = rospy.Publisher('/base_control/back/encoders',StampedEncoders,queue_size=10)
    rospy.loginfo("Node Initiated")

    # Initialize Encoder Message.
    enc_msg_front = StampedEncoders()
    enc_msg_back  = StampedEncoders()
    enc_msg_front.encoders.time_delta = 0.04
    enc_msg_back.encoders.time_delta = 0.04

    pulses = 100
    # Direction:       0      ,180    ,90    ,270    ,45    ,225    ,135   ,315    ,Left  ,Right
    enc_front_left  = [pulses ,-pulses,pulses,-pulses,pulses,-pulses,0     ,0      ,-pulses,pulses ]
    enc_back_left   = [-pulses,pulses ,pulses,-pulses,0     ,0      ,pulses,-pulses,-pulses,pulses ]
    enc_front_right = [-pulses,pulses ,pulses,-pulses,0     ,0      ,pulses,-pulses,pulses ,-pulses]
    enc_back_right  = [pulses ,-pulses,pulses,-pulses,pulses,-pulses,0     ,0      ,pulses ,-pulses]

    last_publish = rospy.Time.now().to_sec()
    last_change = rospy.Time.now().to_sec()
    index_direction = 0

    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - last_publish > 0.04:
            last_publish = rospy.Time.now().to_sec()
            front_publisher.publish(enc_msg_front)
            back_publisher.publish(enc_msg_back)

        if rospy.Time.now().to_sec() - last_change > 4.487:
            last_change = rospy.Time.now().to_sec()
            index_direction = (index_direction +1) % 10

        enc_msg_front.encoders.left_wheel  = enc_front_left[index_direction]
        enc_msg_back.encoders.left_wheel   = enc_back_left[index_direction]
        enc_msg_front.encoders.right_wheel = enc_front_right[index_direction]
        enc_msg_back.encoders.right_wheel  = enc_back_right[index_direction]

        #Publish Rate.
        rate.sleep()

if __name__ == '__main__':
    try:
        run_encoders_simulation()
    except rospy.ROSInterruptException:
        pass
