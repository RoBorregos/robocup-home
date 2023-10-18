#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import moveit_commander
from sensor_msgs.msg import JointState
import time
import tf

# PICK_GROUPS: [arm] #, whole_body, whole_body_rotational
# ARM_GROUP: "arm"
# ARM_JOINTS: [joint1, joint2, joint3, joint4, joint5, joint6]

class PlanningNode():
    ARM_GROUP = "arm"
    ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    ARM_CALIBRATION = [-1.57, 0.0, -3.1416 / 4, 0, -3.1416 / 4, -2.356]
    ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(self):
        rospy.init_node('planning_node')
        self.pick_group = moveit_commander.MoveGroupCommander(PlanningNode.ARM_GROUP, wait_for_servers = 0)
        self.pick_group.set_goal_orientation_tolerance(0.11)
        self.pick_group.set_goal_position_tolerance(0.01)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
    
    def plan(self, pose):
        self.pick_group.set_pose_target(pose)
        self.pick_group.plan()
        self.pick_group.go(wait=True)
        self.pick_group.stop()
        self.pick_group.clear_pose_targets()

    def run(self):
        while rospy.is_shutdown() == False :
            print('hola')
            print('state', self.pick_group.get_current_joint_values())
            state = self.pick_group.get_current_joint_values()
            time.sleep(1)
            input_left_or_right = input('left or right?')
            ten_degrees = 0.174533
            if input_left_or_right == 'l':
                print('left')
                # state[0] += 0.1 euler degrees
                
                state[0] += ten_degrees
            else:
                print('right')
                state[0] -= ten_degrees
            print('calibración')
            self.move_arm(state)
            time.sleep(1) 
            print('home')
            # self.move_arm(PlanningNode.ARM_HOME)
            time.sleep(1)
            # self.listener.waitForTransform('Base', 'Cam1', rospy.Time(0), rospy.Duration(5.0))
            # (trans, rot) = self.listener.lookupTransform('Base', 'Cam1', rospy.Time(0))
            # rospy.sleep(1)
            # # move to the camera position in x by 0.1
            # print('hola ', trans, ' ', rot)
            # new_cam_pose = PoseStamped()
            # new_cam_pose.header.frame_id = "Base"
            # new_trans = trans
            # new_trans[0] += 0.1
            # new_cam_pose.pose.position = Point()
            # new_cam_pose.pose.position.x = new_trans[0]
            # new_cam_pose.pose.position.y = new_trans[1]
            # new_cam_pose.pose.position.z = new_trans[2]
            # new_cam_pose.pose.orientation = Quaternion(rot[0], rot[1], rot[2], rot[3])

            # self.plan(new_cam_pose)
            # rospy.sleep(1)
            # print('llegue')

    def move_arm(self, joint_values):
        joint_state = JointState()
        joint_state.name = self.ARM_JOINTS
        joint_state.position = joint_values
        # set speed
        self.pick_group.set_max_velocity_scaling_factor(0.1)
        self.pick_group.go(joint_state, wait=True)
        self.pick_group.stop()

p = PlanningNode()
p.run() 

