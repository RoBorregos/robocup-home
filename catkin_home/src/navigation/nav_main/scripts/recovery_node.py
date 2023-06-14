#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from move_base_msgs.msg import MoveBaseActionResult
class ClearCostmap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clear_done','clear_error'])
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    def execute(self, userdata):
        # Code to clear the costmap
        try:
            self.clear_costmaps_service()
            return 'clear_done'
        except Exception as e:
            return 'clear_error'

class BackupAndRetry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['backup_done','backup_error','stand_by'])
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # Publisher for Twist messages
        self.move_base_result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        self.error_detect = False
    def move_base_result_callback(self,msg):
        if msg.status.status == 4:
            self.error_detect = True
        else: 
            self.error_detect = False
    def execute(self, userdata):
        try:
            if self.error_detect:
                # Create a Twist message for moving backwards
                twist_msg = Twist()
                twist_msg.linear.x = -0.1  # Move at a speed of -0.1 m/s

                # Publish the Twist message to move the robot backwards
                start_time = rospy.Time.now()
                while rospy.Time.now() - start_time < rospy.Duration.from_sec(1.0):
                    self.vel_pub.publish(twist_msg)  # Publish the Twist message for 1 second
                stop_msg = Twist()
                self.vel_pub.publish(stop_msg)
                return 'backup_done'
            else:
                return 'stand_by'
        except Exception as e:
            # Code to back up and retry
            return 'backup_done'

class RotateInPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotate_done','rotate_error'])
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # Create a publisher for the cmd_vel topic
    def execute(self, userdata):
        # Code to rotate in place
        try:
            rotate_cmd = Twist()
            rotate_cmd.angular.z = math.radians(123)
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < rospy.Duration.from_sec(1.0):
                self.cmd_vel_pub.publish(rotate_cmd)
            stop_msg = Twist()
            self.vecmd_vel_publ_pub.publish(stop_msg)
            return 'rotate_done'
        except Exception as e:
            # Code to back up and retry
            return 'rotate_error'


class MoveToSafeLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_done','move_error'])
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.safegoal=MoveBaseGoal()
        self.safegoal.target_pose.header.frame_id = "map"
        self.safegoal.target_pose.header.stamp = rospy.Time.now()
        self.safegoal.target_pose.pose.position.x =  4.393825531005859
        self.safegoal.target_pose.pose.position.y =  0.008262157440185547
        self.safegoal.target_pose.pose.position.z = 0
        self.safegoal.target_pose.pose.orientation.x = 0
        self.safegoal.target_pose.pose.orientation.y = 0
        self.safegoal.target_pose.pose.orientation.z =  0.7065820693969727
        self.safegoal.target_pose.pose.orientation.w = 0.7076311111450195
    def execute(self, userdata):
        # Code to move to a safe location
        try:
            self.client.send_goal(self.safegoal)
            wait = self.client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return 'move_error'
            else:
                result = self.client.get_result()
                if result==3:
<<<<<<< HEAD
=======
                    
>>>>>>> 94715a36c22a1f512e261a21bc098c54e7b6bd26

                    return 'move_done'
                else:
                    return 'move_error'
        except Exception as e:
            # Code to back up and retry
            return 'move_error'


def main():
    rospy.init_node('recovery_behavior')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('BACKUP_AND_RETRY', BackupAndRetry(), 
                        transitions={'backup_done':'CLEAR_COSTMAP','backup_error':'ROTATE_IN_PLACE','stand_by':'BACKUP_AND_RETRY'})
        smach.StateMachine.add('CLEAR_COSTMAP', ClearCostmap(), 
                               transitions={'clear_done':'MOVE_TO_SAFE_LOCATION','clear_error':'BACKUP_AND_RETRY'})
        smach.StateMachine.add('ROTATE_IN_PLACE', RotateInPlace(), 
                               transitions={'rotate_done':'MOVE_TO_SAFE_LOCATION','rotate_error':'BACKUP_AND_RETRY'})
        smach.StateMachine.add('MOVE_TO_SAFE_LOCATION', MoveToSafeLocation(), 
<<<<<<< HEAD
                               transitions={'move_done':'BACKUP_AND_RETRY', 'move_error': 'ROTATE_IN_PLACE'})
=======
                               transitions={'move_done':'success', 'move_error': 'ROTATE_IN_PLACE'})
>>>>>>> 94715a36c22a1f512e261a21bc098c54e7b6bd26

    # # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('recovery_behavior', sm, '/SM_ROOT')
    sis.start()

    # Run the state machine
    outcome = sm.execute()

    # # Stop the introspection server
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
