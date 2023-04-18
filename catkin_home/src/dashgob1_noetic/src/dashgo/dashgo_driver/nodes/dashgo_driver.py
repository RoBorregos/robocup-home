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
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField 

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


SERVO_MAX = 180
SERVO_MIN = 0



class Stm32:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12


    
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Stm32 PID controller.
        self.PID_INTERVAL = 1000 / 30
        
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
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print("Connecting to Stm32 on port", self.port, "...")
            # self.port = Serial(port="/dev/port1", baudrate=115200, timeout=0.1, writeTimeout=0.1)
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
            print("Stm32 is ready.")

        except SerialException:
            print("Serial Exception:")
            print(sys.exc_info())
            print("Traceback follows:")
            traceback.print_exc(file=sys.stdout)
            print("Cannot connect to Stm32!")
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
             #print "byte:"+str(byte_count_) +","+ "rece_len:"+str(receive_message_length_)
             if self.byte_count_ >= self.receive_message_length_:
                 self.receive_state_ = self.RECEIVE_CHECK

        elif self.receive_state_ == self.RECEIVE_CHECK:
            #print "checksun:" + str(rx_data) + " " + str(self.receive_check_sum_%255)
            #uc_tmp_, = struct.unpack("B",rx_data)
            #print "checksum:" + str(uc_tmp_) +" " + str((self.receive_check_sum_)%255)
            #if uc_tmp_ == (self.receive_check_sum_)%255:
            if 1:
                self.receive_state_ = self.WAITING_FF
                #print str(binascii.b2a_hex(value))
                #left, right, = struct.unpack('hh', value)
                #print "left:"+str(left)+", right:"+str(right)
                return 1 
            else:
                self.receive_state_ = self.WAITING_FF
        else:
            self.receive_state_ = self.WAITING_FF;
        return 0

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Stm32
        '''
        c = ''
        value = ''
        attempts = 0
        c = self.port.read(1)
        #print str(binascii.b2a_hex(c))
        while self.receiveFiniteStates(c) != 1:
            c = self.port.read(1)
            #print str(binascii.b2a_hex(c))
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
        ''' Thread safe execution of "cmd" on the Stm32 returning a single integer value.
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
           # print("ACK", self.payload_ack, self.payload_ack == b'\x00', self.execute(cmd_str)==1)
           return self.FAIL, 0

    def get_encoder_counts(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x02) + struct.pack("B", 0x03)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           left_enc, right_enc, = struct.unpack('HH', self.payload_args)
           return  self.SUCCESS, left_enc, right_enc
        else:
           return self.FAIL, 0, 0


    def get_sonar_range(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0D) + struct.pack("B", 0x0E)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           sonar0, sonar1, sonar2, sonar3, sonar4, sonar5, = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, sonar0, sonar1, sonar2, sonar3, sonar4, sonar5
        else:
           return self.FAIL, 0, 0, 0, 0, 0, 0

    def get_ir_range(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0F) + struct.pack("B", 0x10)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           ir0, ir1, ir2, ir3, ir4, ir5 = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, ir0, ir1, ir2, ir3, ir4, ir5
        else:
           return self.FAIL, 0, 0, 0, 0, 0, 0, 0

    def reset_IMU(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x41) + struct.pack("B", 0x42)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_imu_val(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x05) + struct.pack("B", 0x06)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #left_enc,right_enc, = struct.unpack('hh', self.payload_args)
           yaw, yaw_vel, x_acc, y_acc, z_acc, = struct.unpack('5H', self.payload_args)
           return  self.SUCCESS, yaw, yaw_vel, x_acc, y_acc, z_acc
        else:
           return self.FAIL, 0, 0, 0, 0, 0


    def get_emergency_button(self):
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           emergency_state, _, = struct.unpack('2H', self.payload_args)
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
            #print i, list[i]
            cs += list[i]
        cs=cs%255
        return cs

    def drive(self, left, right):
        data1 = struct.pack("h", left)
        d1, d2 = struct.unpack("BB", data1)

        data2 = struct.pack("h", right)
        c1, c2 = struct.unpack("BB", data2)

        self.check_list = [0x05,0x04, d1, d2, c1, c2]
        self.check_num = self.get_check_sum(self.check_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x05, 0x04) + struct.pack("hh", left, right) + struct.pack("B", self.check_num)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def get_firmware_version(self):
        ''' Get the current version of the firmware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x01) + struct.pack("B", 0x02)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1

    def get_hardware_version(self):
        ''' Get the current version of the hardware.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x13) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0,val1,val2,val3 = struct.unpack('BBBB', self.payload_args)
           return  self.SUCCESS, val0, val1,val2,val3
        else:
           return self.FAIL, -1, -1

    def set_pid(self, cmd, left, right):
        ''' set pid.
        '''
        lpid = int(left*100)
        lpid_l = lpid & 0xff
        lpid_h = (lpid >> 8) & 0xff
        rpid = int(right*100)
        rpid_l = rpid & 0xff
        rpid_h = (rpid >> 8) & 0xff
        check_number_list = [0x05, cmd, lpid_h, lpid_l, rpid_h, rpid_l]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("8B", self.HEADER0, self.HEADER1, 0x05, cmd, lpid_h, lpid_l, rpid_h, rpid_l) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS 
        else:
           return self.FAIL

    def get_pid(self, cmd):
        ''' Get the current value of the imu.
        '''
        check_number_list = [0x01, cmd]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, cmd) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val_l,val_r = struct.unpack('HH', self.payload_args)
           lreal=float(val_l)/100.0
           rreal=float(val_r)/100.0
           return  self.SUCCESS, lreal, rreal
        else:
           return self.FAIL, -1, -1

    def get_infrared(self, ir_id):
        ir_list = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05]
        check_number_list = [0x03, 0x0E, ir_list[ir_id], 0x00]
        checknum = self.get_check_sum(check_number_list)
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x0E, ir_list[ir_id], 0x00) + struct.pack("B", checknum)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
            num,val = struct.unpack('HH', self.payload_args)
            return  self.SUCCESS, num, val
        else:
            return self.FAIL, ir_id, -1

    def get_infrareds(self):
        ''' Get the current distance on the infrareds.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x0F) + struct.pack("B", 0x10)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val0, val1, val2, val3, val4, val5 = struct.unpack('HHHHHH', self.payload_args)
           return  self.SUCCESS, val0, val1, val2, val3, val4, val5
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1

    def get_voltage(self):
        ''' Get the current voltage the battery.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x12) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           vol1, vol2, vol3, vol4, vol5, vol6 = struct.unpack('6H', self.payload_args)
           return  self.SUCCESS, vol1, vol2, vol3, vol4, vol5, vol6
        else:
           return self.FAIL, -1, -1, -1, -1, -1, -1


    def start_automatic_recharge(self):
        ''' start for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x01, 0x00) + struct.pack("B", 0x14)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("start")
           return  self.SUCCESS
        else:
           return self.FAIL

    def stop_automatic_recharge(self):
        ''' stop for automatic recharge.
        '''
        cmd_str=struct.pack("6B", self.HEADER0, self.HEADER1, 0x03, 0x10, 0x00, 0x00) + struct.pack("B", 0x13)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           print("stop")
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_automatic_recharge_status(self):
        ''' Get the status of automatic recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x11) + struct.pack("B", 0x12)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           val, = struct.unpack('I', self.payload_args)
           return self.SUCCESS, val 
        else:
           return self.FAIL, -1

    def get_embtn_recharge(self):
        ''' Get the status of the emergency button and recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x15) + struct.pack("B", 0x16)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           em,rech = struct.unpack('HH', self.payload_args)
           return  self.SUCCESS, em, rech
        else:
           return self.FAIL, -1, -1

    def reset_system(self):
        ''' reset system.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x40) + struct.pack("B", 0x41)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           return  self.SUCCESS
        else:
           return self.FAIL

    def get_recharge_way(self):
        ''' Get the way of the recharge.
        '''
        cmd_str=struct.pack("4B", self.HEADER0, self.HEADER1, 0x01, 0x17) + struct.pack("B", 0x18)
        if (self.execute(cmd_str))==1 and self.payload_ack == b'\x00':
           #print("payload:"+str(binascii.b2a_hex(self.payload_args)))
           way, = struct.unpack('I', self.payload_args)
           return  self.SUCCESS, way
        else:
           return self.FAIL, -1


""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, Stm32, base_frame):
        self.Stm32 = Stm32
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
        self.useImu = rospy.get_param("~useImu", False)
        self.useSonar = rospy.get_param("~useSonar", False)

        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.1518)
        self.wheel_track = rospy.get_param("~wheel_track", 0.375)
        self.encoder_resolution = rospy.get_param("~encoder_resolution", 42760)
        self.gear_reduction = rospy.get_param("~gear_reduction", 1.0)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
       
        self.start_rotation_limit_w = rospy.get_param("~start_rotation_limit_w", 0.4) 
        # Set up PID parameters and check for missing values
        #self.setup_pid(pid_params)
            
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
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscriptions
        #rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("smoother_cmd_vel", Twist, self.cmdVelCallback)
        self.robot_cmd_vel_pub = rospy.Publisher('robot_cmd_vel', Twist, queue_size=5)
        
        # Clear any old odometry info
        self.Stm32.reset_encoders()
        self.Stm32.reset_IMU()
        
        rospy.Subscriber("is_passed", Int16, self.isPassedCallback)
        self.isPassed = True

        rospy.Subscriber("is_passed_2", Int16, self.isPassedCallback_2)
        self.isPassed_2 = True

        ## sonar 
        self.sonar0_pub = rospy.Publisher('sonar0', Range, queue_size=5)
        self.sonar1_pub = rospy.Publisher('sonar1', Range, queue_size=5)
        self.sonar2_pub = rospy.Publisher('sonar2', Range, queue_size=5)
        self.sonar3_pub = rospy.Publisher('sonar3', Range, queue_size=5)
        self.sonar4_pub = rospy.Publisher('sonar4', Range, queue_size=5)
      
        self.sonar_r0 =0.0
        self.sonar_r1 =0.0
        self.sonar_r2 =0.0
        self.sonar_r3 =0.0
        self.sonar_r4 =0.0
        
        self.safe_range_0 = 10
        self.safe_range_1 = 30
        self.sonar_pub_cloud = rospy.Publisher("/sonar_cloudpoint", PointCloud2, queue_size=5)

        self.sonar_height = rospy.get_param("~sonar_height", 0.15)
        self.sonar_maxval = 3.5
        
        #self.robot_radius = rospy.get_param("~robot_radius", 0.21)
        self.point_offset = rospy.get_param("~point_offset", 0.08)
        self.sonar0_offset_yaw = rospy.get_param("~sonar0_offset_yaw", 0.0)
        self.sonar0_offset_x = rospy.get_param("~sonar0_offset_x", 0.27)
        self.sonar0_offset_y = rospy.get_param("~sonar0_offset_y", 0.19)

        self.sonar1_offset_yaw = rospy.get_param("~sonar1_offset_yaw", 0.0)
        self.sonar1_offset_x = rospy.get_param("~sonar1_offset_x", 0.27)
        self.sonar1_offset_y = rospy.get_param("~sonar1_offset_y",-0.19)


        self.sonar2_offset_yaw = rospy.get_param("~sonar2_offset_yaw", 1.57)
        self.sonar2_offset_x = rospy.get_param("~sonar2_offset_x", 0.24)
        self.sonar2_offset_y = rospy.get_param("~sonar2_offset_y", 0.15)

        self.sonar3_offset_yaw = rospy.get_param("~sonar3_offset_yaw", -1.57)
        self.sonar3_offset_x = rospy.get_param("~sonar3_offset_x", 0.24)
        self.sonar3_offset_y = rospy.get_param("~sonar3_offset_y", -0.15)

        self.sonar4_offset_yaw = rospy.get_param("~sonar4_offset_yaw", 3.14)
        self.sonar4_offset_x = rospy.get_param("~sonar4_offset_x", -0.1)
        self.sonar4_offset_y = rospy.get_param("~sonar4_offset_y",  0)


        self.sonar_cloud = [[100.0,0.105,0.1],[100.0,-0.105,0.1],[0.2,100.0,0.1],[0.2,-100.0,0.1],[-100.0,0.0,0.1]]

        self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
        self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
        self.sonar_cloud[0][2] = self.sonar_height

        self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
        self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
        self.sonar_cloud[1][2] = self.sonar_height

        self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
        self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
        self.sonar_cloud[2][2] = self.sonar_height

        self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
        self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
        self.sonar_cloud[3][2] = self.sonar_height
        
        self.sonar_cloud[4][0] = self.sonar4_offset_x + self.sonar_maxval * math.cos(self.sonar4_offset_yaw)
        self.sonar_cloud[4][1] = self.sonar4_offset_y + self.sonar_maxval * math.sin(self.sonar4_offset_yaw)
        self.sonar_cloud[4][2] = self.sonar_height

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
        self.lVelPub = rospy.Publisher('Lvel', Int16, queue_size=5)
        self.rVelPub = rospy.Publisher('Rvel', Int16, queue_size=5)
        #self.Stm32.BEEP_SOUND()

        self.SUCCESS = 0
        self.FAIL = -1

        self.voltage_val = 0
        self.voltage_pub = rospy.Publisher('voltage_value', Int32, queue_size=5)
        self.voltage_percentage_pub = rospy.Publisher('voltage_percentage', Int32, queue_size=5)
        self.voltage_str = ""
        self.voltage_str_pub = rospy.Publisher('voltage_str', String, queue_size=5)
   
        self.emergencybt_val = 0
        self.emergencybt_pub = rospy.Publisher('emergencybt_status', Int16, queue_size=5)
        self.recharge_ir_pub = rospy.Publisher('recharge_ir_status', Int16, queue_size=5)

        rospy.Subscriber("recharge_handle", Int16, self.handleRechargeCallback)
        self.is_recharge = False
        self.recharge_status = 0
        self.recharge_pub = rospy.Publisher('recharge_status', Int16, queue_size=5)

        rospy.Subscriber("imu_reset", Int16, self.resetImuCallback)
        self.imu_angle_pub = rospy.Publisher('imu_angle_Stm32', Int16, queue_size=5)
        self.imu_pub = rospy.Publisher('imu_val', String, queue_size=5)

        rospy.Subscriber("ware_version_req", Int16, self.reqVersionCallback)
        self.version_pub = rospy.Publisher('ware_version', String, queue_size=5)

        self.pid_p_pub = rospy.Publisher('pid_p', String, queue_size=5)
        self.pid_i_pub = rospy.Publisher('pid_i', String, queue_size=5)
        self.pid_d_pub = rospy.Publisher('pid_d', String, queue_size=5)
        rospy.Subscriber("pid_req", String, self.reqPidCallback)
        rospy.Subscriber("pid_set", String, self.reqSetPidCallback)

        rospy.Subscriber("encoder_reset", Int16, self.resetEncoderCallback)
        rospy.Subscriber("system_reset", Int16, self.resetSystemCallback)

        self.recharge_way=0
        self.recharge_way_pub = rospy.Publisher('recharge_way', Int32, queue_size=5)
        self.lwheel_ele = 0
        self.rwheel_ele = 0
        self.lwheel_ele_pub = rospy.Publisher('lwheel_ele', Int32, queue_size=5)
        self.rwheel_ele_pub = rospy.Publisher('rwheel_ele', Int32, queue_size=5)

        self.ir0_pub = rospy.Publisher('ir0', Int32, queue_size=5)
        self.ir1_pub = rospy.Publisher('ir1', Int32, queue_size=5)
        self.ir2_pub = rospy.Publisher('ir2', Int32, queue_size=5)
        self.ir3_pub = rospy.Publisher('ir3', Int32, queue_size=5)
        self.ir4_pub = rospy.Publisher('ir4', Int32, queue_size=5)
        self.ir5_pub = rospy.Publisher('ir5', Int32, queue_size=5)

        self.stm32_version=0
        _,stm32_hardware1,stm32_hardware0,stm32_software1,stm32_software0=self.Stm32.get_hardware_version()
        self.slam_project_version = rospy.get_param("~slam_project_version",0)
        rospy.loginfo ("*************************************************")
        rospy.loginfo ("stm32 hardware_version is "+str(stm32_hardware0)+str(".")+str(stm32_hardware1))
        rospy.loginfo ("stm32 software_version is "+str(stm32_software0)+str(".")+str(stm32_software1))
        rospy.loginfo ("slam version is "+str(self.slam_project_version))
        rospy.loginfo ("*************************************************")

    def handleRechargeCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.start_automatic_recharge()
                self.is_recharge = True
            except:
                rospy.logerr("start automatic recharge exception ")
        else:
            try:
                res = self.Stm32.stop_automatic_recharge()
                self.is_recharge = False
            except:
                rospy.logerr("stop automatic recharge exception ")

    def resetEncoderCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_encoders()
                if res==self.FAIL:
                    rospy.logerr("reset encoder failed ")
            except:
                rospy.logerr("request to reset encoder exception ")

    def resetSystemCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_system()
                if res==self.FAIL:
                    rospy.logerr("reset system failed ")
            except:
                    rospy.logerr("request to reset system exception ")

    def resetImuCallback(self, req):
        if req.data==1:
            try:
                res = self.Stm32.reset_imu()
                if res==self.FAIL:
                    rospy.logerr("reset imu failed ")
            except:
                rospy.logerr("request to reset imu exception ")

    def reqVersionCallback(self, req):
        if req.data==1:
            try:
                res,ver0,ver1,ver2,ver3 = self.Stm32.get_hardware_version()
                self.version_pub.publish(str(ver0)+"."+str(ver1)+"-"+str(ver2)+"."+str(ver3))
                if res==self.FAIL:
                    rospy.logerr("request the version of hardware failed ")
            except:
                self.version_pub.publish("")
                rospy.logerr("request the version of hardware exception ")
        if req.data==2:
            try:
                res,ver0,ver1,ver2,ver3 = self.Stm32.get_firmware_version()
                self.version_pub.publish(str(ver0)+"."+str(ver1)+"-"+str(ver2)+"."+str(ver3))
                if res==self.FAIL:
                    rospy.logerr("request the version of firmware failed ")
            except:
                self.version_pub.publish("")
                rospy.logerr("request the version of firmware exception ")

    def reqPidCallback(self, req):
        if req.data=='P':
            try:
                res,pl,pr = self.Stm32.get_pid(0x07)
                self.pid_p_pub.publish(str(pl) + "," + str(pr))
                if res==self.FAIL:
                    rospy.logerr("request the P of PID failed ")
            except:
                    self.pid_p_pub.publish("")
                    rospy.logerr("request the P of PID exception ")
        if req.data=='I':
            try:
                res,il,ir = self.Stm32.get_pid(0x09)
                self.pid_i_pub.publish(str(il) + "," + str(ir))
                if res==self.FAIL:
                    rospy.logerr("request the I of PID failed ")
            except:
                    self.pid_i_pub.publish("")
                    rospy.logerr("request the I of PID exception ")
        if req.data=='D':
            try:
                res,dl,dr = self.Stm32.get_pid(0x0B)
                self.pid_d_pub.publish(str(dl) + "," + str(dr))
                if res==self.FAIL:
                    rospy.logerr("request the D of PID failed ")
            except:
                self.pid_d_pub.publish("")
                rospy.logerr("request the D of PID exception ")

    def reqSetPidCallback(self, req):
        if req.data!='':
            set_list=req.data.split(",")
            if set_list[0]=='P':
                try:
                    res=self.Stm32.set_pid(0x06, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        rospy.logerr("set the P of PID failed ")
                except:
                    rospy.logerr("set the P of PID exception ")
            if set_list[0]=='I':
                try:
                    res=self.Stm32.set_pid(0x08, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        rospy.logerr("set the I of PID failed ")
                except:
                    rospy.logerr("set the I of PID exception ")
            if set_list[0]=='D':
                try:
                    res=self.Stm32.set_pid(0x0A, float(set_list[1]), float(set_list[2]))
                    if res==self.FAIL:
                        rospy.logerr("set the D of PID failed ")
                except:
                    rospy.logerr("set the D of PID exception ")
        
    def volTransPerentage(self, vo):
         if(vo == -1):
             return -1;
         if(vo>4.2*7*1000):
             COUNT = 10*1000
         else:
             COUNT = 7*1000

         '''
         if(vo >= 3.8*COUNT):
             return 100
         elif(vo >= 3.765*COUNT):
             return 95
         elif(vo >= 3.73*COUNT):
             return 90
         elif(vo >= 3.695*COUNT):
             return 85
         elif(vo >= 3.66*COUNT):
             return 80
         elif(vo >= 3.625*COUNT):
             return 75
         elif(vo >= 3.59*COUNT):
             return 70
         elif(vo >= 3.555*COUNT):
             return 65
         elif (vo >= 3.52*COUNT):
             return 60
         elif (vo >= 3.485*COUNT):
             return 55
         elif (vo >= 3.45*COUNT):
             return 50
         elif (vo >= 3.415*COUNT):
             return 45
         elif (vo >= 3.38*COUNT):
             return 40
         elif (vo >= 3.345*COUNT):
             return 35
         elif (vo >= 3.31*COUNT):
             return 30
         elif (vo >= 3.275*COUNT):
             return 25
         elif (vo >= 3.24*COUNT):
             return 20
         elif (vo >= 3.205*COUNT):
             return 15
         elif (vo >= 3.17*COUNT):
             return 10
         elif (vo >= 3.135*COUNT):
             return 5
         else:
             return 0
         '''
         if(vo >= 4.0*COUNT):
             return 100
         elif(vo >= 3.965*COUNT):
             return 95
         elif(vo >= 3.93*COUNT):
             return 90
         elif(vo >= 3.895*COUNT):
             return 85
         elif(vo >= 3.86*COUNT):
             return 80
         elif(vo >= 3.825*COUNT):
             return 75
         elif(vo >= 3.79*COUNT):
             return 70
         elif(vo >= 3.755*COUNT):
             return 65
         elif (vo >= 3.72*COUNT):
             return 60
         elif (vo >= 3.685*COUNT):
             return 55
         elif (vo >= 3.65*COUNT):
             return 50
         elif (vo >= 3.615*COUNT):
             return 45
         elif (vo >= 3.58*COUNT):
             return 40
         elif (vo >= 3.545*COUNT):
             return 35
         elif (vo >= 3.51*COUNT):
             return 30
         elif (vo >= 3.475*COUNT):
             return 25
         elif (vo >= 3.44*COUNT):
             return 20
         elif (vo >= 3.405*COUNT):
             return 15
         elif (vo >= 3.37*COUNT):
             return 10
         elif (vo >= 3.335*COUNT):
             return 5
         else:
             return 0
        

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                stat_, left_enc,right_enc = self.Stm32.get_encoder_counts()#
                #rospy.loginfo("left_enc:  " + str(left_enc)+" right_enc: " + str(right_enc))
                self.lEncoderPub.publish(left_enc)
                self.rEncoderPub.publish(right_enc)
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                      
            try:
                res,vol1,vol2,vol3,vol4,vol5,vol6 = self.Stm32.get_voltage()
                self.lwheel_ele = vol3*4
                self.lwheel_ele_pub.publish(self.lwheel_ele)
                self.rwheel_ele = vol4*4
                self.rwheel_ele_pub.publish(self.rwheel_ele)
                self.voltage_val = vol5
                self.voltage_pub.publish(self.voltage_val)
                self.voltage_str_pub.publish(str(vol1) + "," + str(vol2) + "," + str(vol3) + "," + str(vol4) + "," + str(vol5) + "," + str(vol6))
                self.voltage_percentage_pub.publish(self.volTransPerentage(self.voltage_val))
                #rospy.loginfo("voltage_Perentage:" + str(self.volTransPerentage(self.voltage_val)))
            except:
                self.voltage_pub.publish(-1)
                self.voltage_str_pub.publish("")
                self.lwheel_ele_pub.publish(-1)
                self.rwheel_ele_pub.publish(-1)
                rospy.logerr("get voltage exception")

            try:
                res,ir1,ir2,ir3,ir4,ir5,ir6 = self.Stm32.get_infrareds()
                self.ir0_pub.publish(ir1)
                self.ir1_pub.publish(ir2)
                self.ir2_pub.publish(ir3)
                self.ir3_pub.publish(ir4)
                self.ir4_pub.publish(ir5)
                self.ir5_pub.publish(ir6)
            except:
                self.ir0_pub.publish(-1)
                self.ir1_pub.publish(-1)
                self.ir2_pub.publish(-1)
                self.ir3_pub.publish(-1)
                self.ir4_pub.publish(-1)
                self.ir5_pub.publish(-1)
                rospy.logerr("get infrared ray exception")

            try:
                res,way = self.Stm32.get_recharge_way()
                self.recharge_way = way
                #self.recharge_way_pub.publish(ways)
                self.recharge_way_pub.publish(self.recharge_way)
            except Exception as e:
                #print str(e)
                self.recharge_way_pub.publish(-1)
                rospy.logerr("get recharge_way  exception")

            try:
                res,status = self.Stm32.get_automatic_recharge_status()
                self.recharge_status = status
                if self.recharge_status == 3:
                    self.is_recharge = False
                self.recharge_pub.publish(self.recharge_status)
            except:
                self.recharge_pub.publish(-1)
                rospy.logerr("get recharge_status  exception")

            if (not self.is_recharge):
                try:
                    res,eme_val,rech_val  = self.Stm32.get_embtn_recharge()
                    self.emergencybt_val = eme_val
                    self.emergencybt_pub.publish(eme_val)
                    self.recharge_ir_pub.publish(rech_val)
                except:
                    self.emergencybt_val = -1
                    self.emergencybt_pub.publish(-1)
                    self.recharge_ir_pub.publish(-1)
                    rospy.logerr("get emergencybt  exception")

            if (self.useSonar == True) :
                pcloud = PointCloud2()
                try:
                    stat_, self.sonar_r0, self.sonar_r1, self.sonar_r2, self.sonar_r3, self.sonar_r4,_ = self.Stm32.get_sonar_range()
                    #self.sonar_r3=80 
                    #rospy.loginfo("sonar0: " + str(self.sonar_r0)+" sonar1: " + str(self.sonar_r1)+" sonar2: " + str(self.sonar_r2)+" sonar3: " + str(self.sonar_r3)+" sonar4: " + str(self.sonar_r4))  

                    sonar0_range = Range()
                    sonar0_range.header.stamp = now
                    sonar0_range.header.frame_id = "/sonar0"
                    sonar0_range.radiation_type = Range.ULTRASOUND
                    sonar0_range.field_of_view = 0.3
                    sonar0_range.min_range = 0.04
                    sonar0_range.max_range = 0.8
                    sonar0_range.range = self.sonar_r0/100.0
                    #if sonar0_range.range>=sonar0_range.max_range or sonar0_range.range == 0.0:
                    if sonar0_range.range == 0.0:  #sonar0 error or not exist flag
                        sonar0_range.range=1.0
                    elif sonar0_range.range>=sonar0_range.max_range:
                        sonar0_range.range = sonar0_range.max_range
                    self.sonar0_pub.publish(sonar0_range)
                    if sonar0_range.range>=0.5 or sonar0_range.range == 0.0:
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
                    else: 
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + sonar0_range.range * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + sonar0_range.range * math.sin(self.sonar0_offset_yaw)


                    sonar1_range = Range()
                    sonar1_range.header.stamp = now
                    sonar1_range.header.frame_id = "/sonar1"
                    sonar1_range.radiation_type = Range.ULTRASOUND
                    sonar1_range.field_of_view = 0.3
                    sonar1_range.min_range = 0.04
                    sonar1_range.max_range = 0.8 
                    sonar1_range.range = self.sonar_r1/100.0
                   # if sonar1_range.range>=sonar0_range.max_range or sonar1_range.range == 0.0:
                    if sonar1_range.range == 0.0:  #sonar1 error or not exist flag
                        sonar1_range.range=1.0
                    elif sonar1_range.range>=sonar0_range.max_range:
                        sonar1_range.range = sonar1_range.max_range
                    self.sonar1_pub.publish(sonar1_range) 
                    if sonar1_range.range>=0.5 or sonar1_range.range == 0.0:
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
                    else: 
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + sonar1_range.range * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + sonar1_range.range * math.sin(self.sonar1_offset_yaw)

                    sonar2_range = Range()
                    sonar2_range.header.stamp = now
                    sonar2_range.header.frame_id = "/sonar2"
                    sonar2_range.radiation_type = Range.ULTRASOUND
                    sonar2_range.field_of_view = 0.3
                    sonar2_range.min_range = 0.04
                    sonar2_range.max_range = 0.8
                    sonar2_range.range = self.sonar_r2/100.0
                    #if sonar2_range.range>=sonar2_range.max_range or sonar2_range.range == 0.0:
                    if sonar2_range.range == 0.0:  #sonar2 error or not exist flag
                        sonar2_range.range=1.0
                    elif sonar2_range.range>=sonar2_range.max_range :
                        sonar2_range.range = sonar2_range.max_range
                    self.sonar2_pub.publish(sonar2_range)
                    if sonar2_range.range>=0.5 or sonar2_range.range == 0.0:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
                    else:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + sonar2_range.range * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + sonar2_range.range * math.sin(self.sonar2_offset_yaw)

                    sonar3_range = Range()
                    sonar3_range.header.stamp = now
                    sonar3_range.header.frame_id = "/sonar3"
                    sonar3_range.radiation_type = Range.ULTRASOUND
                    sonar3_range.field_of_view = 0.3
                    sonar3_range.min_range = 0.04
                    sonar3_range.max_range = 0.8
                    sonar3_range.range = self.sonar_r3/100.0
                    #if sonar3_range.range>=sonar3_range.max_range or sonar3_range.range == 0.0:
                    if sonar3_range.range == 0.0:  #sonar3 error or not exist flag
                        sonar3_range.range=1.0
                    elif sonar3_range.range>=sonar3_range.max_range :
                        sonar3_range.range = sonar3_range.max_range
                    self.sonar3_pub.publish(sonar3_range)
                    if sonar3_range.range>=0.5 or sonar3_range.range == 0.0:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
                    else:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + sonar3_range.range * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + sonar3_range.range * math.sin(self.sonar3_offset_yaw)

                    sonar4_range = Range()
                    sonar4_range.header.stamp = now
                    sonar4_range.header.frame_id = "/sonar4"
                    sonar4_range.radiation_type = Range.ULTRASOUND
                    sonar4_range.field_of_view = 0.3
                    sonar4_range.min_range = 0.04
                    sonar4_range.max_range = 0.8
                    sonar4_range.range = self.sonar_r4/100.0
                    #if sonar4_range.range>=sonar4_range.max_range or sonar4_range.range == 0.0:
                    if sonar4_range.range == 0.0:  #sonar4 error or not exist flag
                        sonar4_range.range=1.0
                    elif sonar4_range.range>=sonar4_range.max_range :
                        sonar4_range.range = sonar4_range.max_range
                    self.sonar4_pub.publish(sonar4_range)
                    if sonar4_range.range>=0.5 or sonar4_range.range == 0.0:
                        self.sonar_cloud[4][0] = self.sonar4_offset_x + self.sonar_maxval * math.cos(self.sonar4_offset_yaw)
                        self.sonar_cloud[4][1] = self.sonar4_offset_y + self.sonar_maxval * math.sin(self.sonar4_offset_yaw)
                    else:
                        self.sonar_cloud[4][0] = self.sonar4_offset_x + sonar4_range.range * math.cos(self.sonar4_offset_yaw)
                        self.sonar_cloud[4][1] = self.sonar4_offset_y + sonar4_range.range * math.sin(self.sonar4_offset_yaw)

                    pcloud.header.frame_id="/base_footprint"
                    pcloud = pc2.create_cloud_xyz32(pcloud.header, self.sonar_cloud)
                    self.sonar_pub_cloud.publish(pcloud)
        
                except:
                    rospy.logerr("Get Sonar exception")
                    return

            #try:
            #    stat_, ir0, ir1, ir2, ir3, ir4, ir5 = self.Stm32.get_ir_range()
            #    #rospy.loginfo("ir0: " + str(ir0)+" ir1: " + str(ir1)+" ir2: " + str(ir2)+" ir3: " + str(ir3)+" ir4: " + str(ir4)+" ir5: " + str(ir5))            
            #except:
            #    self.bad_encoder_count += 1
            #    rospy.logerr("Sonar exception count: " + str(self.bad_encoder_count))

            if (self.useImu == True) :
                try:
                    stat_, yaw, yaw_vel, acc_x, acc_y, acc_z = self.Stm32.get_imu_val()
                    if yaw>=18000:
                        yaw = yaw-65535
                    yaw = yaw/100.0
                    if yaw_vel>=32768:
                        yaw_vel = yaw_vel-65535
                    yaw_vel = yaw_vel/100.0
                    #rospy.loginfo("yaw: " + str(yaw/100)+" yaw_vel: " + str(yaw_vel))     
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
                    imu_quaternion = Quaternion()
                    imu_quaternion.x = 0.0 
                    imu_quaternion.y = 0.0
                    imu_quaternion.z = sin(-1*self.imu_offset*yaw*3.1416/(180 *2.0))
                    imu_quaternion.w = cos(-1*self.imu_offset*yaw*3.1416/(180 *2.0))
                    imu_data.orientation = imu_quaternion
                    imu_data.linear_acceleration_covariance[0] = -1
                    imu_data.angular_velocity_covariance[0] = -1

                    imu_data.angular_velocity.x = 0.0
                    imu_data.angular_velocity.y = 0.0
                    imu_data.angular_velocity.z = (yaw_vel*3.1416/(180*100))
                    self.imuPub.publish(imu_data)
                    self.imuAnglePub.publish(-1*self.imu_offset*yaw*3.1416/(180 *2.0))
       
                except:
                    self.bad_encoder_count += 1
                    rospy.logerr("IMU exception count: " + str(self.bad_encoder_count))
                    return
            

                      
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                if (left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap) :
                    self.l_wheel_mult = self.l_wheel_mult + 1     
                elif (left_enc > self.encoder_high_wrap and self.enc_left < self.encoder_low_wrap) :
                    self.l_wheel_mult = self.l_wheel_mult - 1
                else:
                     self.l_wheel_mult = 0
                if (right_enc < self.encoder_low_wrap and self.enc_right > self.encoder_high_wrap) :
                    self.r_wheel_mult = self.r_wheel_mult + 1     
                elif (right_enc > self.encoder_high_wrap and self.enc_right < self.encoder_low_wrap) :
                    self.r_wheel_mult = self.r_wheel_mult - 1
                else:
                     self.r_wheel_mult = 0
                #dright = (right_enc - self.enc_right) / self.ticks_per_meter
                #dleft = (left_enc - self.enc_left) / self.ticks_per_meter
                dleft = 1.0 * (left_enc + self.l_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_left) / self.ticks_per_meter 
                dright = 1.0 * (right_enc + self.r_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_right) / self.ticks_per_meter 

            self.enc_right = right_enc
            self.enc_left = left_enc
            
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
            # todo sensor_state.distance == 0
            #if self.v_des_left == 0 and self.v_des_right == 0:
            #    odom.pose.covariance = ODOM_POSE_COVARIANCE2
            #    odom.twist.covariance = ODOM_TWIST_COVARIANCE2
            #else:
            #    odom.pose.covariance = ODOM_POSE_COVARIANCE
            #    odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            self.lVelPub.publish(self.v_left)
            self.rVelPub.publish(self.v_right)            

            # Set motor speeds in encoder ticks per PID loop
            #if not self.stopped:
            if ((not self.stopped) and (not self.is_recharge)):
                self.Stm32.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.Stm32.drive(0, 0)

    def isPassedCallback(self, msg):
        if(msg.data>2):
            self.isPassed = False
        else:
            self.isPassed = True

    def isPassedCallback_2(self, msg):
        if(msg.data>2):
            self.isPassed_2 = False
        else:
            self.isPassed_2 = True
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        robot_cmd_vel = Twist()
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if self.emergencybt_val==1:
            robot_cmd_vel.linear.x = 0
            robot_cmd_vel.linear.y = 0
            robot_cmd_vel.angular.z = 0
        else:
            robot_cmd_vel.linear.x = x
            robot_cmd_vel.linear.y = 0
            robot_cmd_vel.angular.z = th
        self.robot_cmd_vel_pub.publish(robot_cmd_vel)

        if not self.isPassed and x>0 :
            x = 0

        if not self.isPassed_2 and x>0 :
            x = 0

        if (self.useSonar == True) :
            #sonar0
            if((self.sonar_r0<=self.safe_range_0 and self.sonar_r0>=2) and (x<0)):
                x= 0.0
                rospy.logwarn("sonar0 smaller than safe_range_0, cannot back")
            #sonar1
            if((self.sonar_r1<=self.safe_range_0 and self.sonar_r1>=2) and (x>0)):
                x=0.0
                th=0.2
                rospy.logwarn("sonar1 smaller than safe_range_0, only trun left")
            
            if((self.sonar_r1<=self.safe_range_0 and self.sonar_r1>=2) and (th<0)):
                x=0.0
                th=0.2
            #sonar2
            if((self.sonar_r2<=self.safe_range_0 and self.sonar_r2>=2) and (x>0)):
                x=0.0
                th=0.2
                rospy.logwarn("sonar2 smaller than safe_range_0, only trun left")

            #sonar3
            if((self.sonar_r3<=self.safe_range_0 and self.sonar_r3>=2) and (x>0)):
                x=0.0
                th=-0.2
                rospy.logwarn("sonar3 smaller than safe_range_0, only trun left")

            if((self.sonar_r3<=self.safe_range_0 and self.sonar_r3>=2) and (th>0)):
                x=0.0
                th=-0.2
            #sonar4
            if((self.sonar_r4<=self.safe_range_0 and self.sonar_r0>=2) and (x<0)):
                x= 0.0
                rospy.logwarn("sonar4 smaller than safe_range_0, cannot back")



        if x == 0:
            # Turn in place
            #print "1111bianhuaqian x=0 , w= "+str(th)
            if th>0.0 and th<0.15:
                th=0.15
            elif th>-0.15 and th<0.0:
                th=-0.15
            #print "1111gaibianhou w= "+str(th)
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            #print "222bianhua w=0  v="+str(x)
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            #print "3333bianhua qian v,w is not 0, v= "+str(x)+ " w= "+str(th)
            '''
            if (th>0.0 and th<0.2) and (x>-0.05 and x<0.05):
                th=0.2
            if (th>-0.2 and th<0.0) and (x>-0.05 and x<0.05):
                th=-0.2
            '''
            if (th>0.0 and th<self.start_rotation_limit_w) and (x>-0.1 and x<0):
                th=self.start_rotation_limit_w
            if (th<0.0 and th >-1.0*self.start_rotation_limit_w) and (x>-0.1 and x<0):
                th=-1.0*self.start_rotation_limit_w


            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.Stm32.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.Stm32.PID_RATE)


class Stm32ROS():
    def __init__(self):
        rospy.init_node('Stm32', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_footprint')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", True)
        
        
        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Initialize the controlller
        self.controller = Stm32(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Stm32 on port " + self.port + " at " + str(self.baud) + " baud")
     
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
        rospy.loginfo("Shutting down Stm32 Node...")
        
if __name__ == '__main__':
    myStm32 = Stm32ROS()
