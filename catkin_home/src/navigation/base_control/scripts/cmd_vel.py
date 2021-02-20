#!/usr/bin/env python
'''
    Script that simulate navigation stack action of sending velocity commands.
'''
import rospy
from geometry_msgs.msg import Twist

def run_cmd_velocity():
    '''
        Main function that simultate the action of sending velocity commands.
    '''
    # Starts a new node.
    rospy.init_node('cmd_velocity', anonymous=True)
    rate = rospy.Rate(4) # ROS Rate at 4Hz.
    velocity_publisher = rospy.Publisher('/base_control/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Node Initiated")

    # Initialize Twist Message.
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    speed = 0.18
    # Direction: 0    , 45   , 90   , 135   , 180   , 225   , 270   , 315     Left   Right
    x_values=    [0    , speed, speed, speed , 0     , -speed, -speed, -speed, 0    , 0     ]
    y_values=    [speed, speed, 0    , -speed, -speed, -speed, 0     , speed , 0    , 0     ]
    z_values=    [0    , 0    , 0    , 0     , 0     ,  0    ,  0    , 0     , speed, -speed]

    state = 0
    last_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        if rospy.Time.now().to_sec() - last_time > 5:
            state = (state+1) % len(x_values)
            last_time = rospy.Time.now().to_sec()
            rospy.loginfo("Velocity Changing")

        vel_msg.linear.x  = x_values[state]
        vel_msg.linear.y  = y_values[state]
        vel_msg.angular.z = z_values[state]
        velocity_publisher.publish(vel_msg)

        # Publish Rate.
        rate.sleep()

if __name__ == '__main__':
    try:
        run_cmd_velocity()
    except rospy.ROSInterruptException:
        pass
