#!/usr/bin/env python3
import time,os,math,struct
from math import pi

from serial import Serial
from serial.serialutil import SerialException

import rospy
from std_msgs.msg import Float32MultiArray

#rate = 0.05

#x_max = 479.0
#x_min = -638.0
#y_max = 624.0
#y_min = -598.0
#z_max = 547.0
#z_min = -556.0


class Magnetometer:
    def __init__(self, port="/dev/port5", baudrate=115200, timeout=0.5, writeTimeout=0.5):
        self.angle = Float32MultiArray()
        self.angle.data.append(0.0)
        self.angle.data.append(0.0)
        self.flag = True
        self.data = []

        self.skip_num = 0
        self.skip_max = 5
        self.RAD_TO_DEG = 57.29577951308232087679815481410
        #self.magOffset = [(x_max+x_min)/2, (y_max+y_min)/2, (z_max+z_min)/2]

        rospy.init_node('magnetometer', anonymous=True)
        self.magnetometer = rospy.Publisher('magnetometer', Float32MultiArray, queue_size=1)
        
        try:
            self.port = Serial(port=port, baudrate=baudrate, timeout=timeout, writeTimeout=writeTimeout)
            time.sleep(0.1)
        except SerialException:
            print("Serial Exception")
            os._exit(1)

        #cmd_init = (struct.pack("4B",0xD6,0x6D,0x01,0x01))  #init
        #cmd_stop_attitude = (struct.pack("4B",0xD6,0x6D,0x25,0x25))  #stop attitude
        #cmd_start_magnetometer = (struct.pack("4B",0xD6,0x6D,0x17,0x17))  #start geomagntic_field
        #self.send(cmd)
        #time.sleep(0.2)

        
    def open(self): 
        self.port.open()

    def close(self): 
        self.port.close() 
    
    def send(self, cmd):
        self.port.write(cmd)

    def read(self):
        return self.port.read()

    def show(self):
        cmd=0x73
        while not rospy.is_shutdown():
            self.data = []
            self.data.append(struct.unpack('B', self.read())[0])

            if self.data[0] == 0xa7:
                for i in range(0,3):
                    self.data.append(struct.unpack('B', self.read())[0])

                if self.data[1]==0x7a and self.data[2]==cmd and self.data[3]==0x07:
                    for i in range(0,7):
                        self.data.append(struct.unpack('B', self.read())[0])
                    
                    x = self.data[6] <<8 | self.data[5]
                    y = self.data[8] <<8 | self.data[7]
                    z = self.data[10]<<8 | self.data[9]
                    if x>2000:
                        x=x-65535
                    if y>2000:
                        y=y-65535
                    if z>2000:
                        z=z-65535

                    #self.angle.data[1] = -math.atan2((y-self.magOffset[0]),(x-self.magOffset[1]))# + pi
                    self.angle.data[1] = -math.atan2(y,x)# + pi
                    if self.skip_num<self.skip_max:
                        self.skip_num=self.skip_num+1
                        continue
                    if self.flag:
                        self.angle.data[0] = self.angle.data[1]
                        self.flag = False
                    self.magnetometer.publish(self.angle)
                    #print (self.angle.data[1]+pi) * self.RAD_TO_DEG
            #time.sleep(rate)

    def __del__(self):
        self.port.close()

if __name__ == '__main__':
    Magnetometer().show()
    rospy.spin()
