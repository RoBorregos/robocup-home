#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('cmd_velocity', anonymous=True)
    rate = rospy.Rate(4) # ROS Rate at 4Hz
    velocity_publisher = rospy.Publisher('/base_control/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("Node Initiated");
    
    #Initialize Twist Message
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    speed = 0.35
    #Direction:  0    , 45   , 90   , 135   , 180   , 225   , 270   , 315     Left   Right   
    xValues=    [0    , speed, speed, speed , 0     , -speed, -speed, -speed, 0    , 0     ]
    yValues=    [speed, speed, 0    , -speed, -speed, -speed, 0     , speed , 0    , 0     ]
    zValues=    [0    , 0    , 0    , 0     , 0     ,  0    ,  0    , 0     , speed, -speed]

    state = 0 
    t0 = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
    
        if rospy.Time.now().to_sec() - t0 > 3:
            state = (state+1) % len(xValues)
            t0 = rospy.Time.now().to_sec()


        vel_msg.linear.x  = xValues[state]
        vel_msg.linear.y  = yValues[state]
        vel_msg.angular.z = zValues[state]

        velocity_publisher.publish(vel_msg)
        
        #Publish Rate
        rate.sleep()
        

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass