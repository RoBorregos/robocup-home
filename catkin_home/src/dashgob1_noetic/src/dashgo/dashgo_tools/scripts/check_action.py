#!/usr/bin/env python3

from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist, Quaternion, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import radians, pi, copysign, copysign, sqrt, pow

import tf
import PyKDL
import actionlib
import dashgo_tools.msg

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

class CheckServer():
    
    # create messages that are used to publish feedback/result
    #_goal = dashgo_tools.msg.check_msgGoal()
    _feedback = dashgo_tools.msg.check_msgFeedback()
    _result = dashgo_tools.msg.check_msgResult()

    def __init__(self):
        
        #print("loginfo: CheckServer init()")
        self._as = actionlib.SimpleActionServer('check_server', dashgo_tools.msg.check_msgAction, self.server_callback, False)
        self._as.start()
        self._ac = actionlib.SimpleActionClient('check_server', dashgo_tools.msg.check_msgAction)
        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)     
        rospy.Subscriber("check", String, self.topic_callback)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

    def topic_callback(self,msg_data):

        print
        print("loginfo: topic_callback()")
        _msg = msg_data.data.split()
        print("topic parameters: %s" %_msg)
        check_flag = True

        if _msg[0].upper() == "STOP":
            rospy.loginfo("Stopping the robot...")
            goal_pub = dashgo_tools.msg.check_msgGoal(method="STOP",imu="FALSE", distance=0,angule=0, vel=0, error=10)
            self._ac.wait_for_server()
            self._ac.send_goal(goal_pub)
            return

        if len(_msg)==5:
            _msg[0] = _msg[0].upper()
            _msg[1] = _msg[1].upper()
            if _msg[0] not in ("LINE","LINEAR","ANGULE","ANGULAR"):
                check_flag = False
                print("Error: For \"LINEAR, ANGULAR\", 5 parameters expected, %s received." %len(_msg))
                print("Parameters: method, imu, distance/angule, velocity, tolerance")
            if _msg[1] not in ("TRUE","1","FALSE","0"):
                check_flag = False
                print("Error: 1st parameter should be \"true\" or \"false\" only, using imu or not.")
            try:
                _msg[2]=float(_msg[2])
                _msg[3]=float(_msg[3])
                _msg[4]=float(_msg[4])
            except ValueError:
                check_flag = False
                print("Error: No.3-5 parameters should be float type, meaning goal, vel, tolerance")
        elif len(_msg)==6:
            _msg[0] = _msg[0].upper()
            _msg[1] = _msg[1].upper()
            if _msg[0] not in ("CYCLE","ARC"):
                check_flag = False
                print("Error: For \"CYCLE, ARC\", 6 parameters expected, %s received." %len(_msg))
                print("Parameters: method, imu, distanceradius, angule, velocity, tolerance")
            if _msg[1] not in ("TRUE","1","FALSE","0"):
                check_flag = False
                print("Error: 1st parameter should be \"true\" or \"false\" only, using imu or not.")
            try:
                _msg[2]=float(_msg[2])
                _msg[3]=float(_msg[3])
                _msg[4]=float(_msg[4])
                _msg[5]=float(_msg[5])
            except ValueError:
                check_flag = False
                print("Error: No.3-6 parameters should be float type, meaning goal, vel, tolerance")
        else:
            check_flag = False
            print("Error: For 5 or 6 parameters expected, %s received." %len(_msg))

        if check_flag:
            if _msg[0] in ("LINE","LINEAR"):
                goal_pub = dashgo_tools.msg.check_msgGoal(method=_msg[0],imu=_msg[1],distance=_msg[2],angule=0,vel=_msg[3],error=_msg[4])
            elif _msg[0] in ("ANGULE","ANGULAR"):
                goal_pub = dashgo_tools.msg.check_msgGoal(method=_msg[0],imu=_msg[1],distance=0,angule=_msg[2],vel=_msg[3],error=_msg[4])
            else:
                goal_pub = dashgo_tools.msg.check_msgGoal(method=_msg[0],imu=_msg[1],distance=_msg[2],angule=_msg[3],vel=_msg[4],error=_msg[5])
            self._ac.wait_for_server()
            self._ac.send_goal(goal_pub)
        else:
            print("waiting for next commend")
        return
    
    def server_callback(self,goal):

        print("loginfo: server_callback()")

        if goal.method=="STOP":
            self.stop_execute()
        elif goal.angule==0:
            if (abs(goal.distance)>10 or abs(goal.vel)<0.02 or abs(goal.vel)>2.0):
                print("Warning: make sure the distance < 10m, the velocity is between 0.02~2.0 m/s.")
                self.cmd_vel.publish(Twist())
                self._result.issuccess = True
                self._as.set_succeeded(self._result)
                print("waiting for next commend")
                return
            self.linear_execute(goal)
        elif goal.distance==0:
            if (abs(goal.vel)<0.02 or abs(goal.vel)>pi):
                print("Warning: make sure the velocity is between 0.02~pi rad/s.")
                self.cmd_vel.publish(Twist())
                self._result.issuccess = True
                self._as.set_succeeded(self._result)
                print("waiting for next commend")
                return
            self.angular_execute(goal)
        else:
            if (goal.distance<0 or abs(goal.vel)<0.02 or abs(goal.vel)>2):
                print("Warning: make sure the radius > 0m, the velocity is between 0.02~2 m/s.")
                self.cmd_vel.publish(Twist())
                self._result.issuccess = True
                self._as.set_succeeded(self._result)
                print("waiting for next commend")
                return
            self.cycle_execute(goal)
        return

    def stop_execute(self):

        rospy.loginfo("stop_execute(): Stopping the robot...")
        self.cmd_vel.publish(Twist())
        self._result.issuccess = True
        self._as.set_succeeded(self._result)
        print("waiting for next commend")

    def linear_execute(self, goal):

        print("loginfo: linear_execute()")

        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        # Set the distance to travel
        self.test_distance = rospy.get_param('~test_distance', goal.distance) # meters
        self.speed = rospy.get_param('~speed', goal.vel) # meters per second
        self.tolerance = rospy.get_param('~tolerance', goal.error) # meters
        if self.test_distance<0:
            self.speed = -self.speed
        self.odom_linear_scale_correction = rospy.get_param('~odom_linear_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)

        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        if goal.imu.upper() in ("FALSE","0"):
            self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        else:
            self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))        
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
  
        self.position = Point()
        
        # Get the starting position from the tf transform between the odom and base frames
        self.position = self.get_position()
        
        x_start = self.position.x
        y_start = self.position.y
            
        move_cmd = Twist()

        while not rospy.is_shutdown():
            # Stop the robot by default
            move_cmd = Twist()

            if self._as.is_preempt_requested():
                rospy.loginfo('check_linear: Preempted')
                self._as.set_preempted()
                break

            if self.start_test:
                # Get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()
                
                # Compute the Euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))
                if self.test_distance<0:
                    distance = -distance
                # Correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction
                
                # How close are we?
                error =  distance - self.test_distance
                
                # Are we close enough?
                if not self.start_test or abs(error) <  self.tolerance:
                    self.start_test = False
                    params = {'start_test': False}
                    # rospy.loginfo(params)

                    self._result.issuccess = True
                    rospy.loginfo('linear_execute: Succeeded')
                    self._as.set_succeeded(self._result)
                    break
                else:
                    # If not, move in the appropriate direction
                    move_cmd.linear.x = copysign(self.speed, -1 * error)

                    self._feedback.accomplished = distance
                    self._as.publish_feedback(self._feedback)

            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")

    def angular_execute(self, goal):

        print("loginfo: angular_execute() called")
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        # The test angle is 360 degrees
        self.test_angle = radians(rospy.get_param('~test_angle', goal.angule))
        self.speed = rospy.get_param('~speed', goal.vel) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', goal.error)) # degrees converted to radians
        if goal.angule<0:
            self.speed = -self.speed
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        if goal.imu.upper() in ("FALSE","0"):
            self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        else:
            self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        
        while not rospy.is_shutdown():

            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    if self._as.is_preempt_requested():
                        rospy.loginfo('check_angular: Preempted')
                        self._as.set_preempted()
                        self.cmd_vel.publish(Twist())
                        print("waiting for next commend")
                        return

                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    self._feedback.accomplished = turn_angle * 57.2957795 # 180/pi
                    self._as.publish_feedback(self._feedback)

                    # Compute the new error
                    error = self.test_angle - turn_angle

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle

                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}

                self._result.issuccess = True
                rospy.loginfo('angular_execute: Succeeded')
                self._as.set_succeeded(self._result)
                break
                
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")
        return

    def cycle_execute(self, goal): 

        print("loginfo: cycle_execute() called")
        
        # How fast will we check the odometry values?
        self.rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(self.rate)

        # The test angle is 360 degrees
        #self.test_distance = rospy.get_param('~test_distance', self._goal.distance) # meters
        self.test_angle = radians(rospy.get_param('~test_angle', goal.angule))
        self.speed = rospy.get_param('~speed', goal.vel) # radians per second
        self.tolerance = radians(rospy.get_param('tolerance', goal.error)) # degrees converted to radians
        self.odom_angular_scale_correction = rospy.get_param('~odom_angular_scale_correction', 1.0)
        self.start_test = rospy.get_param('~start_test', True)
        
        # Publisher to control the robot's speed
        #self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
 
        # The base frame is usually base_link or base_footprint
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')

        # The odom frame is usually just /odom
        if goal.imu.upper() in ("FALSE","0"):
            self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        else:
            self.odom_frame = rospy.get_param('~odom_frame', '/odom_combined')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Make sure we see the odom and base frames
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
            
        rospy.loginfo("Bring up rqt_reconfigure to control the test.")
        
        reverse = 1
        
        while not rospy.is_shutdown():

            if self.start_test:
                # Get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle
                                
                # Alternate directions between tests
                reverse = -reverse
                
                while abs(error) > self.tolerance and self.start_test:
                    if rospy.is_shutdown():
                        return
                    
                    if self._as.is_preempt_requested():
                        rospy.loginfo('check_angular: Preempted')
                        self._as.set_preempted()
                        self.cmd_vel.publish(Twist())
                        print("waiting for next commend")
                        return

                    # Rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.linear.x = copysign(self.speed, error)
                    move_cmd.angular.z = move_cmd.linear.x / goal.distance
                    if goal.angule<0:
                        move_cmd.angular.z = -move_cmd.angular.z
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                 
                    # Get the current rotation angle from tf                   
                    self.odom_angle = self.get_odom_angle()
                    
                    # Compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)
                    
                    # Add to our total angle so far
                    turn_angle += delta_angle

                    self._feedback.accomplished = turn_angle * 57.2957795 # 180/pi
                    self._as.publish_feedback(self._feedback)

                    # Compute the new error
                    error = self.test_angle - turn_angle
                    if goal.angule<0:
                        error = -error

                    # Store the current angle for the next comparison
                    last_angle = self.odom_angle

                # Stop the robot
                self.cmd_vel.publish(Twist())
                
                # Update the status flag
                self.start_test = False
                params = {'start_test': False}

                self._result.issuccess = True
                rospy.loginfo('cycle_execute: Succeeded')
                self._as.set_succeeded(self._result)
                break
                
            rospy.sleep(0.5)
                    
        # Stop the robot
        self.cmd_vel.publish(Twist())
        print("waiting for next commend")

    def get_position(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)

    def get_odom_angle(self):
            # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))
            
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    print("")
    print("check_server begin")
    rospy.init_node('check_server')
    server = CheckServer()
    rospy.spin()
