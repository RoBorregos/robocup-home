#!/usr/bin/env python3

import rospy
import tf
#from math import pi as PI,  radians, sin, cos
import math
from sensor_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32,PointStamped
from std_msgs.msg import Float32
import sensor_msgs.point_cloud2 as pc2

class GetScanData():
    def __init__(self):
        rospy.init_node('get_scan_data',anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.tf_listener = tf.TransformListener()
        #print "0000000000000000000000000000"
        self.tf_listener.waitForTransform("/map", "laser_frame", rospy.Time(), rospy.Duration(10.0))
        
        self.scan_pub_cloud = rospy.Publisher("/android_scan", PointCloud,queue_size=5)
        self.rate =rospy.get_param('rate',20)
        r=rospy.Rate(self.rate)
        rospy.Subscriber("/scan",LaserScan,self.callback)

        
        rospy.spin()

    def callback(self,req):
        #print "scan angle min: "+str(req.angle_min)
        #print "scan angle max: "+str(req.angle_max)
        #print "scan data len ="+str(len(req.ranges))
        #self.andriod_scan_pcloud=[]
        self.pcloud = PointCloud()
        self.count=0
        self.laser_filter_range=[]
        scan_data_len=len(req.ranges)
        self.pcloud.header.frame_id="/map"
        self.laser_point=PointStamped()
        self.laser_point.header.frame_id="laser_frame"
        self.laser_point.point.x=0
        self.laser_point.point.y=0
        self.laser_point.point.z=0
        for i in range(scan_data_len):
            current_point_angle=req.angle_min+i*req.angle_increment
            current_point_distance=req.ranges[i]
            #if current_point_distance <=2:       
            self.laser_point.point.x = math.cos(current_point_angle)*current_point_distance
            self.laser_point.point.y = math.sin(current_point_angle)*current_point_distance
            self.map_point=self.tf_listener.transformPoint("map",self.laser_point)
            map_x = self.map_point.point.x
            map_y = self.map_point.point.y            
            p=Point32()
            p.x=map_x
            p.y=map_y
            p.z=0

            self.pcloud.points.append(p)

        #self.pcloud = pc2.create_cloud_xyz32(self.pcloud.header, self.andriod_scan_pcloud)
        self.scan_pub_cloud.publish(self.pcloud)

    def shutdown(self):
        rospy.loginfo("stop the robot")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        GetScanData()
    except:
        rospy.loginfo("GetScanData terminated")


