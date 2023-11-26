#!/usr/bin/env python3
'''
This script creates a socket Server for communication with another 
computer. Used to establish connection between ROS Noetic Master 
and Navigation module developed in ROS Kinetic - EAI Dashgo B1.
'''
import time
import socket
import sys
import rospy
from std_msgs.msg import String

class NavBridgeServer(object):
    DEBUG = True
    CLIENT = ('192.168.31.80', 8009)
    SERVER = ('192.168.31.123', 8009)
    BUFFER = 4096

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(self.SERVER)
        
        self.talker = rospy.Subscriber("navBridgeServer/talker", String, self.sendMsg)
        self.listener = rospy.Publisher("navBridgeServer/listener", String, queue_size=20)
        try:
            self.startConection()
        except KeyboardInterrupt:
            self.sock.close()
    
    def startConection(self):
        while not rospy.is_shutdown():
            self.sock.settimeout(None)
            while True:
                buf = self.sock.recv(self.BUFFER)
                if not buf:
                    self.sock.close()
                    break
                self.listener.publish(String(buf.decode("utf-8")))

    def sendMsg(self, msg):
        self.sock.sendto(msg.data.encode(), self.CLIENT)
    

def main():
    rospy.init_node('navBridgeServer', anonymous=0)
    rospy.loginfo('navBridgeServer initialized.')
    NavBridgeServer()
    rospy.spin()

if __name__ == '__main__':
    main()

