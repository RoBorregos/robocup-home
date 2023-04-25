#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import numpy as np
import math
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
import time

"""
Class to make FK calls using the /compute_fk service.


Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GetFK(object):
    def __init__(self, fk_link, frame_id):
        """
        A class to do FK calls thru the MoveIt!'s /compute_ik service.

        :param str fk_link: link to compute the forward kinematics
        :param str frame_id: frame_id to compute the forward kinematics
        into account collisions
        """
        rospy.loginfo("Initalizing GetFK...")
        self.fk_link = fk_link
        self.frame_id = frame_id
        rospy.loginfo("Asking forward kinematics for link: " + self.fk_link)
        rospy.loginfo("PoseStamped answers will be on frame: " + self.frame_id)
        self.fk_srv = rospy.ServiceProxy('/compute_fk',
                                         GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Connected!")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

    def js_cb(self, data):
        self.last_js = data

    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        print("Got a joint state message!",end="\r") 
        print(self.last_js,end="\r")
        return self.get_fk(self.last_js)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.

        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if fk_link is None:
            fk_link = self.fk_link

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.fk_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code.val = 99999  # Failure
            return resp


if __name__ == '__main__':
    rospy.init_node('test_fk')
    rospy.loginfo("Querying for FK")
    
    gfk = GetFK('end_effector_link', 'base_link')
    resp = gfk.get_current_fk()
    
    pubPose = rospy.Publisher('fk_pose', PoseStamped, queue_size=10)
    pubPoseArray = rospy.Publisher('fk_poses', PoseArray, queue_size=10)
    x = PoseArray()
    x.header = resp.pose_stamped[0].header
    x.poses = []
    x.poses.append(resp.pose_stamped[0].pose)
    
    time.sleep(1)
    pubPose.publish(resp.pose_stamped[0])
    pubPoseArray.publish(x)
    
    print(gfk.get_current_fk_pose())
    exit()
    # Simulate joint[1-5]
    joint_limits = [
        (-2.0 * math.pi, 2.0 * math.pi),
        (-2.059, 2.0944),
        (-3.927, 0.19198),
        (-1.69297, math.pi),
        (-2.0 * math.pi, 2.0 * math.pi)
    ]
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
    
    # Calculate Operations
    count = 1
    for i in range(len(joint_limits)):
        count = count * (joint_limits[i][1] - joint_limits[i][0]) / 1
    print("Total Operations: ", count)
    last_time = time.time()
    for i in np.arange(joint_limits[0][0], joint_limits[0][1], 0.5):
        for j in np.arange(joint_limits[1][0], joint_limits[1][1], 0.5):
            for k in np.arange(joint_limits[2][0], joint_limits[2][1], 0.5):
                for l in np.arange(joint_limits[3][0], joint_limits[3][1], 0.5):
                    for m in np.arange(joint_limits[4][0], joint_limits[4][1], 0.5):
                        joint_state = JointState()
                        joint_state.name = joint_names
                        joint_state.position = [i, j, k, l, m]
                        resp = gfk.get_fk(joint_state)
                        if resp.error_code and resp.error_code.val == 1:
                            x.poses.append(resp.pose_stamped[0].pose)
                        if time.time() - last_time > 10:
                            last_time = time.time()
                            pubPoseArray.publish(x)
                            print("Updating")
    

    rospy.loginfo(resp)