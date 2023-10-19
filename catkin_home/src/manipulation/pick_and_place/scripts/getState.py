#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import tf

class GetState():
    ARM_GROUP = "arm"
    def __init__(self):
        rospy.init_node('get_state', anonymous=True)
        self.moveit_commander = moveit_commander.MoveGroupCommander(GetState.ARM_GROUP, wait_for_servers = 0)
        self.moveit_commander.set_goal_orientation_tolerance(0.11)
        self.moveit_commander.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
    
    def run(self):
        while rospy.is_shutdown() == False and input("cont: (y/n) ") != 'n':
            print('state', self.moveit_commander.get_current_joint_values())

if __name__ == '__main__':
    try:
        node = GetState()
        node.run()
    except rospy.ROSInterruptException:
        pass
