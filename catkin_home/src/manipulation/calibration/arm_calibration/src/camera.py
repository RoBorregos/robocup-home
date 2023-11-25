#!/usr/bin/env python3

import rospy
import actionlib
from tf import transformations
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header
from arm_server.msg import MoveArmAction, MoveArmGoal
import numpy as np
import tf
import moveit_commander
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET
import threading
import time


ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')
URDF_PATH = 'src/robot_description/urdf/dashgo.urdf.xacro'

tree = ET.parse(URDF_PATH, ET.XMLParser(encoding='utf-8'))
# print tree info to check if it has loaded correctly
print(ET.tostring(tree.getroot(), encoding='utf-8', method='xml'))
root = tree.getroot()
camera_tag = root.find(".//xacro:load_camera[@name='Cam1']", namespaces={"xacro": "http://ros.org/wiki/xacro"})

class CameraPosePublisher:

    def __init__(self):
        self.ARM_CALIBRATION = [-1.57, 0.0, -3.1416 / 4, 0, -3.1416 / 4, -2.356]
        self.ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rospy.init_node('camera_pose_publisher')

        self.move_arm_client = actionlib.SimpleActionClient('/move_arm_as', MoveArmAction)
        self.move_arm_client.wait_for_server()

        # Set up TF listener and broadcaster
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.publishRotationFixedTransform = True
        
        self.arm_goal = MoveArmGoal()
        self.arm_goal.joints_target = self.ARM_CALIBRATION
        self.arm_goal.speed = 0.2

        self.move_arm_client.send_goal(self.arm_goal)
        self.move_arm_client.wait_for_result()
        print( self.move_arm_client.get_result() )

        time.sleep(8.5) # Wait for an accurate detection.
        try:
            # Get the transforms between 'base_footprint', 'apriltag', and 'tag_5'
            print("Waiting for transforms...")
            self.listener.waitForTransform('Base', 'apriltag', rospy.Time(0), rospy.Duration(5.0))
            self.listener.waitForTransform('Base', 'tag_5', rospy.Time(0), rospy.Duration(5.0))
            self.listener.waitForTransform('Base', 'Cam1', rospy.Time(0), rospy.Duration(5.0))
            self.listener.waitForTransform('Cam1', 'tag_5', rospy.Time(0), rospy.Duration(5.0))
            print("Got transforms!")
            (trans1, rot1) = self.listener.lookupTransform('Base', 'apriltag', rospy.Time(0))
            (trans2, rot2) = self.listener.lookupTransform('Base', 'tag_5', rospy.Time(0))
            (trans3, rot3) = self.listener.lookupTransform('Base', 'Cam1', rospy.Time(0))
            (trans4, rot4) = self.listener.lookupTransform('Cam1', 'tag_5', rospy.Time(0))
            print("Got matrices!")
            # Fix Rotation Difference between 'apriltag' and 'tag_5'
            rot_diff = transformations.quaternion_multiply(rot1, transformations.quaternion_inverse(rot2))
            rot_diff = transformations.quaternion_multiply(rot_diff, rot3)
            rot_fixed = rot_diff

            # Publish the camera transform with the rotation fixed.
            broadcasterThread = threading.Thread(target=self.send_transform, args=(trans3, rot_fixed, trans4, rot4))
            broadcasterThread.start()
            # Get the transform between 'Base' and 'tag_fixed' (the detection with the fixed rotation).
            self.listener.waitForTransform('Base', 'tag_fixed', rospy.Time(0), rospy.Duration(5.0))
            (trans6, _) = self.listener.lookupTransform('Base', 'tag_fixed', rospy.Time(0))
            self.publishRotationFixedTransform = False
            broadcasterThread.join()

            # Calculate the translation difference between the detection with the rotation fixed and the ground truth.
            trans_diff = np.subtract(trans1, trans6)
            trans_cam = np.add(trans3, trans_diff)

            # Publish the resultant transform
            self.broadcaster.sendTransform(trans_cam, rot_fixed, rospy.Time.now(), 'Cam1', 'Base')
                
            # Update the URDF file with the new camera transform.
            rot_euler_fixed = transformations.euler_from_quaternion(rot_fixed)
            if camera_tag is not None:
                print("Found camera tag!", camera_tag)
                camera_tag.set('x', str(trans_cam[0]))
                print('x = ', camera_tag.get('x'))
                camera_tag.set('y', str(trans_cam[1]))
                print('y = ', camera_tag.get('y'))
                camera_tag.set('z', str(trans_cam[2]))
                print('z = ', camera_tag.get('z'))
                camera_tag.set('r_r', str(rot_euler_fixed[1]))
                print('r_r = ', camera_tag.get('r_r'))
                camera_tag.set('r_p', str(rot_euler_fixed[0]))
                print('r_p = ', camera_tag.get('r_p'))
                camera_tag.set('r_y', str(rot_euler_fixed[2]))
                print('r_y = ', camera_tag.get('r_y'))
            tree.write(URDF_PATH, encoding='utf-8', xml_declaration=True)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get transforms")

        self.arm_goal.joints_target = self.ARM_HOME 
        self.move_arm_client.send_goal(self.arm_goal)
        self.move_arm_client.wait_for_result()
        print( self.move_arm_client.get_result() )


    def send_transform(self, trans, rot, detection_trans, detection_rot):
        start = time.time()
        while(time.time() - start < 5 and self.publishRotationFixedTransform):
            self.broadcaster.sendTransform(trans, rot, rospy.Time.now(), 'New_Cam1', 'Base')
            self.broadcaster.sendTransform(detection_trans, detection_rot, rospy.Time.now(), 'tag_fixed', 'New_Cam1') 
   
if __name__ == '__main__':
    CameraPosePublisher()
