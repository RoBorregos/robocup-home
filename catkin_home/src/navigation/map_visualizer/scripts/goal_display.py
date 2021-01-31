#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import csv
import tf
import os

path = []
markerArray = MarkerArray()

def getPath():
    global path
    # declare the coordinates of interest
    with open(rospy.get_param('~path_data'), 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            path.append((float(row[0]), float(row[1]), 0.1))
        rospy.loginfo(path)

def updateMarkers():
    global path
    global markerArray
    markerArray = MarkerArray()
    counter=0
    for position in path:
        marker = Marker()
        marker.header.frame_id = "/goal" + str(counter)
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.ns = "map_goal_point"
        marker.id = counter
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        markerArray.markers.append(marker)
        counter+=1
    rospy.loginfo(str(counter) + " markers added to display!")


if __name__ == '__main__':
    topic = 'goal_visualization_array'
    publisher = rospy.Publisher(topic, MarkerArray)
    br = tf.TransformBroadcaster()
    rospy.init_node('path_displayer')

    getPath()
    global path

    while not rospy.is_shutdown():
        counter=0
        for position in path:
            br.sendTransform(position,
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "/goal" + str(counter),
                            "/map")
            counter+=1
        updateMarkers()
        global markerArray
        publisher.publish(markerArray)
        rospy.sleep(0.01)