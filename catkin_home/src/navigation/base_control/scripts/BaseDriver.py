#!/usr/bin/env python3
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist
import os, time
import _thread

import math
from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

import roslib

from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int32, UInt16, Float32, String
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Range, Imu

from tf.transformations import quaternion_from_euler

import struct
import binascii
 
ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


class Microcontroller:
    ''' Configuration Parameters
    '''
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        self.WAITING_FF = 0
        self.WAITING_AA = 1
        self.RECEIVE_LEN = 2
        self.RECEIVE_PACKAGE = 3
        self.RECEIVE_CHECK = 4
        self.HEADER0 = 0xff
        self.HEADER1 = 0xaa
        
        self.SUCCESS = 0
        self.FAIL = -1

        self.receive_state_ = self.WAITING_FF
        self.receive_check_sum_ = 0
        self.payload_command = b''
        self.payload_ack = b''
        self.payload_args = b''
        self.payload_len = 0
        self.byte_count_ = 0
        self.receive_message_length_ = 0
    
        # Keep things thread safe
        self.mutex = _thread.allocate_lock()

    def connect(self):
        try:
            print("Connecting to Microcontroller on port", self.port, "...")
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            state_, val = self.get_baud()
            if val != self.baudrate:
                time.sleep(1)
                state_, val  = self.get_baud()
                if val != self.baudrate:
                    raise SerialException
            print("Connected at", self.baudrate)
            print("Microcontroller is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Microcontroller!")
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd)


    def receiveFiniteStates(self, rx_data):
        if self.receive_state_ == self.WAITING_FF:
            #print str(binascii.b2a_hex(rx_data))
            if rx_data == b'\xff':
                self.receive_state_ = self.WAITING_AA
                self.receive_check_sum_ =0
                self.receive_message_length_ = 0
                self.byte_count_=0
                self.payload_ack = b''
                self.payload_args = b''
                self.payload_len = 0


        elif self.receive_state_ == self.WAITING_AA :
             if rx_data == b'\xaa':
                 self.receive_state_ = self.RECEIVE_LEN
                 self.receive_check_sum_ = 0
             else:
                 self.receive_state_ = self.WAITING_FF

        elif self.receive_state_ == self.RECEIVE_LEN:
             self.receive_message_length_, = struct.unpack("B",rx_data)
             self.receive_state_ = self.RECEIVE_PACKAGE
             self.receive_check_sum_ = self.receive_message_length_
        elif self.receive_state_ == self.RECEIVE_PACKAGE:
             if self.byte_count_==0:
                 self.payload_ack = rx_data
             else:
                 self.payload_args += rx_data
             uc_tmp_, = struct.unpack("B",rx_data)
             self.receive_check_sum_ = self.receive_check_sum_ + uc_tmp_
             self.byte_count_ +=1
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            self.receive_state_ = self.WAITING_FF
            return 1 
        else:
            self.receive_state_ = self.WAITING_FF
        return 0

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Microcontroller
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        while self.receiveFiniteStates(c) != 1:
            c = self.port.read(1)
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return 0
        return 1
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Microcontroller returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd)
            res = self.recv(self.timeout)
            while attempts < ntries and res !=1 :
                try:
                    self.port.flushInput()
                    self.port.write(cmd)
                    res = self.recv(self.timeout)
                    #print "response : " + str(binascii.b2a_hex(res))
                except:
                    print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
                attempts += 1
        except:
            self.mutex.release()
            print("Exception executing command: " + str(binascii.b2a_hex(cmd)))
            return 0
        
        self.mutex.release()
        return 1
                                 

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x00) + struct.pack("B", 0x01)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val 
        else:
           return self.FAIL, 0

    def get_encoder_counts(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           back_left_encoders, back_right_encoders, front_left_encoders, front_right_encoders, = struct.unpack('hhhh', self.payload_args)
           return  self.SUCCESS, back_left_encoders, back_right_encoders, front_left_encoders, front_right_encoders
        else:
           return self.FAIL, 0, 0

    def reset_IMU(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x41) + struct.pack("B", 0x42)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_imu_val(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x05) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           yaw, yaw_vel, x_acc, y_acc, z_acc, = struct.unpack('5f', self.payload_args)
           return  self.SUCCESS, yaw, yaw_vel, x_acc, y_acc, z_acc
        else:
           return self.FAIL, 0, 0, 0, 0, 0

    def get_emergency_button(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           emergency_state, = struct.unpack('B', self.payload_args)
           return  self.SUCCESS, emergency_state
        else:
           return self.FAIL, 0

    def reset_encoders(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x03) + struct.pack("B", 0x04)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_check_sum(self,list):
        list_len = len(list)
        cs = 0
        for i in range(list_len):
            cs += list[i]
        cs=cs%255
        return cs

    def drive(self, x, y, th):
        # data1 = struct.pack("h", x)
        # data2 = struct.pack("h", th)
        # d1, d2 = struct.unpack("BB", data1)
        # c1, c2 = struct.unpack("BB", data2)
        # self.check_list = [0x05,0x04, d1, d2, c1, c2]
        # self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x0D, 0x04) + struct.pack("fff", x, y, th) + struct.pack("B", 0x05)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def stop(self):
        ''' Stop both motors.
        '''
        return self.drive(0, 0, 0)

    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, val
        else:
           return self.FAIL, -1, -1


""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, Microcontroller, base_frame):
        self.Microcontroller = Microcontroller
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 20))
        self.timeout = rospy.get_param("~base_controller_timeout", 0.1)
        self.stopped = False
        self.useImu = rospy.get_param("~useImu", False)
        
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.1518)
        self.wheel_track = rospy.get_param("~wheel_track", 0.375)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 42760)
        self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
       
        self.start_rotation_limit_w = rospy.get_param("~start_rotation_limit_w", 0.4) 
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        self.encoder_min = rospy.get_param('encoder_min', 0)
        self.encoder_max = rospy.get_param('encoder_max', 65535)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_x = 0                    # cmd_vel setpoint
        self.v_y = 0
        self.v_th = 0
        self.last_cmd_vel = now

        self.emergencybt_val = 0
        self.emergencybt_pub = rospy.Publisher('emergencybt_status', Int16, queue_size=5)

        # Subscriptions
        rospy.Subscriber("smoother_cmd_vel", Twist, self.cmdVelCallback)
        self.robot_cmd_vel_pub = rospy.Publisher('robot_cmd_vel', Twist, queue_size=5)
        
        # Clear any old odometry info
        self.Microcontroller.reset_encoders()
        self.Microcontroller.reset_IMU()

        self.imu_frame_id = rospy.get_param('imu_frame_id', 'imu_base')
        self.imu_offset = rospy.get_param('imu_offset', 1.01)
        self.imuPub = rospy.Publisher('imu', Imu, queue_size=5)
        self.imuAnglePub = rospy.Publisher('imu_angle', Float32, queue_size=5)
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

        self.lEncoderPub = rospy.Publisher('Lencoder', UInt16, queue_size=5)
        self.rEncoderPub = rospy.Publisher('Rencoder', UInt16, queue_size=5)

        self.SUCCESS = 0
        self.FAIL = -1
   
        rospy.Subscriber("imu_reset", Int16, self.resetImuCallback)
        self.imu_angle_pub = rospy.Publisher('imu_angle_MicroController', Int16, queue_size=5)
        self.imu_pub = rospy.Publisher('imu_val', String, queue_size=5)

        rospy.Subscriber("encoder_reset", Int16, self.resetEncoderCallback)

        self.lwheel_ele = 0
        self.rwheel_ele = 0
        self.lwheel_ele_pub = rospy.Publisher('lwheel_ele', Int32, queue_size=5)
        self.rwheel_ele_pub = rospy.Publisher('rwheel_ele', Int32, queue_size=5)

        self.last_time = time.time()
        self.micro_version=0
        _,version=self.Microcontroller.get_hardware_version()
        self.slam_project_version = rospy.get_param("~slam_project_version",0)
        rospy.loginfo ("*************************************************")
        rospy.loginfo ("micro hardware_version is "+str(version)+str(".")+str(version))
        rospy.loginfo ("micro software_version is "+str(version)+str(".")+str(version))
        rospy.loginfo ("slam version is "+str(self.slam_project_version))
        rospy.loginfo ("*************************************************")

    def resetEncoderCallback(self, req):
        if req.data==1:
            try:
                res = self.Microcontroller.reset_encoders()
                if res==self.FAIL:
                    rospy.logerr("reset encoder failed ")
            except:
                rospy.logerr("request to reset encoder exception ")

    def resetImuCallback(self, req):
        if req.data==1:
            try:
                res = self.Microcontroller.reset_imu()
                if res==self.FAIL:
                    rospy.logerr("reset imu failed ")
            except:
                rospy.logerr("request to reset imu exception ")

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                stat_, left_enc, right_enc, _, _ = self.Microcontroller.get_encoder_counts()#
                #rospy.loginfo("left_enc:  " + str(left_enc)+" right_enc: " + str(right_enc))
                self.lEncoderPub.publish(left_enc)
                self.rEncoderPub.publish(right_enc)
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                      
            try:
                self.lwheel_ele = vol3*4
                self.lwheel_ele_pub.publish(self.lwheel_ele)
                self.rwheel_ele = vol4*4
                self.rwheel_ele_pub.publish(self.rwheel_ele)
            except:
                self.lwheel_ele_pub.publish(-1)
                self.rwheel_ele_pub.publish(-1)
                

            try:
                res  = self.Microcontroller.get_emergency_button()
                self.emergencybt_val = res[1]
                self.emergencybt_pub.publish(self.emergencybt_val)
            except:
                self.emergencybt_val = -1
                self.emergencybt_pub.publish(-1)
                rospy.logerr("get emergencybt  exception")


            if (self.useImu == True) :
                try:
                    stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = self.Microcontroller.get_imu_val()
                    # Degree to radians
                    yaw = yaw * 3.1415926 / 180.0
                    isValid = True
                    if yaw == 0.0:
                        isValid = False

                    # print("yaw:  " + str(yaw)+" yaw_vel: " + str(yaw_vel)+" acc_x: " + str(acc_x)+" acc_y: " + str(acc_y)+" acc_z: " + str(acc_z))
                    if isValid:
                        # if yaw>=18000:
                        #     yaw = yaw-65535
                        # yaw = yaw/100.0
                        # if yaw_vel>=32768:
                        #     yaw_vel = yaw_vel-65535
                        # yaw_vel = yaw_vel/100.0
                        
                        imu_data = Imu()  
                        imu_data.header.stamp = rospy.Time.now()
                        imu_data.header.frame_id = self.imu_frame_id 
                        imu_data.orientation_covariance[0] = 1000000
                        imu_data.orientation_covariance[1] = 0
                        imu_data.orientation_covariance[2] = 0
                        imu_data.orientation_covariance[3] = 0
                        imu_data.orientation_covariance[4] = 1000000
                        imu_data.orientation_covariance[5] = 0
                        imu_data.orientation_covariance[6] = 0
                        imu_data.orientation_covariance[7] = 0
                        imu_data.orientation_covariance[8] = 0.000001

                        newquat = quaternion_from_euler(0, 0, yaw)
                        imu_data.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
                        imu_data.linear_acceleration_covariance[0] = -1
                        imu_data.angular_velocity_covariance[0] = -1

                        imu_data.linear_acceleration.x = acc_x
                        imu_data.linear_acceleration.y = acc_y
                        imu_data.linear_acceleration.z = acc_z

                        imu_data.angular_velocity.x = 0.0
                        imu_data.angular_velocity.y = 0.0
                        imu_data.angular_velocity.z = yaw_vel
                        self.imuPub.publish(imu_data)
                        self.imuAnglePub.publish(yaw)
                except:
                    self.bad_encoder_count += 1
                    rospy.logerr("IMU exception count: " + str(self.bad_encoder_count))
                    return
     
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            dright = (1.0 * right_enc) / self.ticks_per_meter
            dleft = (1.0 * left_enc) / self.ticks_per_meter
            # print(dright, dleft, right_enc, left_enc)

            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            if (self.useImu == False) :
                self.odomBroadcaster.sendTransform(
                  (self.x, self.y, 0), 
                  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                  rospy.Time.now(),
                  self.base_frame,
                  "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_x = 0
                self.v_y = 0
                self.v_th = 0

            # Set motor speeds in encoder ticks per PID loop
            if ((not self.stopped)):
                self.Microcontroller.drive(self.v_x, self.v_y, self.v_th)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.Microcontroller.drive(0, 0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        robot_cmd_vel = Twist()
        x = req.linear.x         # m/s
        y = req.linear.y         # m/s
        th = req.angular.z       # rad/s

        if self.emergencybt_val == 1:
            robot_cmd_vel.linear.x = 0
            robot_cmd_vel.linear.y = 0
            robot_cmd_vel.angular.z = 0
        else:
            robot_cmd_vel.linear.x = x
            robot_cmd_vel.linear.y = y
            robot_cmd_vel.angular.z = th
        self.robot_cmd_vel_pub.publish(robot_cmd_vel)

        self.v_x =  robot_cmd_vel.linear.x
        self.v_y =  robot_cmd_vel.linear.y
        self.v_th = robot_cmd_vel.angular.z

class MicroControllerROS():
    def __init__(self):
        rospy.init_node('Microcontroller', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.25)
        self.base_frame = rospy.get_param("~base_frame", 'base_footprint')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)


        self.use_base_controller = rospy.get_param("~use_base_controller", True)
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Initialize the controlller
        self.controller = Microcontroller(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Microcontroller on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = _thread.allocate_lock()
              
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
            r.sleep()
    
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Microcontroller Node...")

def testController():
    # Initialize the controlller
    port = "/dev/ttyUSB0"
    baud = 57600
    timeout = 0.1
    controller = Microcontroller(port, baud, timeout)
    controller.connect()
    rospy.loginfo("Connected to Microcontroller on port " + port + " at " + str(baud) + " baud")
    print("IMU", controller.get_imu_val())
    print(controller.get_hardware_version())
    print(controller.get_baud())
    print("EC", controller.get_encoder_counts())
    print("EC", controller.get_encoder_counts())
    print("IMU", controller.get_imu_val())
    print("EB", controller.get_emergency_button())
    print("EB", controller.get_emergency_button())
    print("EB", controller.get_emergency_button())
    print(controller.reset_encoders())
    
    print(controller.stop())
    
    velocities = [-0.25, 0.25]
    index = 0
    start_time = time.time()
    while(1):
        print(controller.get_imu_val())
        print(controller.get_emergency_button())
        print(controller.get_encoder_counts())
        if time.time() - start_time > 2.5:
            index = (index + 1) % 2
            start_time = time.time()
        controller.drive(velocities[index], 0*velocities[index], 0.0)
        time.sleep(0.1)

if __name__ == '__main__':
    # testController()
    myMicroController = MicroControllerROS()
    
