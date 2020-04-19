#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('cmd_velocity', anonymous=True)
    rate = rospy.Rate(4) # ROS Rate at 4Hz
    velocity_publisher = rospy.Publisher('/base_control/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("Node Initiated");
    
    vel_msg = Twist()

    #Receiveing the user's input
    speed = 0.35
    distance = 1000
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 35
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            ##Publish Rate
            rate.sleep()
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass