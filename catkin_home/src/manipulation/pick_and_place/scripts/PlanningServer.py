#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf

# PICK_GROUPS: [arm] #, whole_body, whole_body_rotational
# ARM_GROUP: "arm"
# ARM_JOINTS: [joint1, joint2, joint3, joint4, joint5, joint6]

class PlanningNode():
    ARM_GROUP = "arm"
    ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def __init__(self):
        self.pick_group = moveit_commander.MoveGroupCommander(PlanningNode.ARM_GROUP, wait_for_servers = 0)
        self.pick_group.set_goal_orientation_tolerance(0.11)
        self.pick_group.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        rospy.init_node('planning_node')
    
    def plan(self, pose):
        self.pick_group.set_pose_target(pose)
        self.pick_group.plan()
        self.pick_group.go(wait=True)
        self.pick_group.stop()
        self.pick_group.clear_pose_targets()

    def run(self):
        self.listener.waitForTransform('Base', 'Cam1', rospy.Time(0), rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform('Base', 'Cam1', rospy.Time(0))
        # move to the camera position in x by 0.1
        new_cam_pose = PoseStamped()
        new_cam_pose.header.frame_id = "Base"

