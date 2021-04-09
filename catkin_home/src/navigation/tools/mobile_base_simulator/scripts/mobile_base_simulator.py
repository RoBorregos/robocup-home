#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import Twist

def callback_cmd_vel(msg):
    global vx, vy, w
    vx = msg.linear.x
    vy = msg.linear.y
    w  = msg.angular.z

def main():
    print "INITIALIZING MOBILE BASE ..."
    rospy.init_node("mobile_base")
    rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel, queue_size=1)
    fs = 30
    loop = rospy.Rate(fs)
    br = tf.TransformBroadcaster()

    global vx, vy, w
    vx, vy, w = 0,0,0

    robot_x = 0
    robot_y = 0
    robot_t = 0

    while not rospy.is_shutdown():
        robot_x += 1.0/fs * (vx*math.cos(robot_t) - vy*math.sin(robot_t))
        robot_y += 1.0/fs * (vx*math.sin(robot_t) + vy*math.cos(robot_t))
        robot_t += 1.0/fs * w
        
        quat = tf.transformations.quaternion_from_euler(0, 0, robot_t)
        current_time = rospy.Time.now()
        br.sendTransform((robot_x, robot_y, 0), quat, current_time, "base_link", "odom")
        
        loop.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
