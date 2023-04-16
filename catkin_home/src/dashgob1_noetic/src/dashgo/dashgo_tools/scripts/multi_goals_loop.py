#!/usr/bin/env python3
#coding=utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import json_file

#多点目标点点击（默认方向）
#导航目标点状态处理
def status_callback(msg):
    global goal_pub, index,markerArray
    global try_again

    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    #                             #    sent over the wire by an action server

    if(msg.status.status == 3):
        try_again = 1
        print 'Goal reached'
        if index == count:
            index = 0

        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = markerArray.markers[index].pose.position.x
        pose.pose.position.y = markerArray.markers[index].pose.position.y
        pose.pose.orientation.w = 1
        goal_pub.publish(pose)
        index += 1
    else:
        
        if try_again == 1:
            print 'Goal cannot reached has some error :',msg.status.status," try again!!!!"
            index = index-1;
            try_again = 0
        else:
            print 'Goal cannot reached has some error :',msg.status.status," again , now go to next goal!!!!"
            if index == len(markerArray.markers):
                index=0

        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = markerArray.markers[index].pose.position.x
        pose.pose.position.y = markerArray.markers[index].pose.position.y
        pose.pose.orientation.w = 1
        goal_pub.publish(pose)
        index += 1

#rviz点击目标点处理
def click_callback(msg):
    global markerArray,count
    global goal_pub,index

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.text = str(count)
    marker.id = len(markerArray.markers);
    markerArray.markers.append(marker)

    # Publish the MarkerArray
    mark_pub.publish(markerArray)

    #first goal
    if count==0:
        pose = PoseStamped()
        pose.header.frame_id = "/map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1
        goal_pub.publish(pose)
        index += 1

    count += 1

    #保存目标点
    goal={"position": {"x":msg.point.x,"y":msg.point.y,"z":msg.point.z}, "quaternion": {"x":0.0,"y":0.0,"z":0.0,"w":1.0}} 
    file_name="multi_goals";
#    json_file.json_write(goal , file_name)
    json_file.json_append(goal , file_name)
#    dic=json_file.json_read_lines(dic , file_name)

    print 'add a path goal point'


markerArray = MarkerArray()
count = 0       #total goal num
index = 0       #current goal point index
try_again = 1  # try the fail goal once again

rospy.init_node('path_point_loop')

mark_pub = rospy.Publisher('/path_point', MarkerArray,queue_size=100)
click_sub = rospy.Subscriber('/clicked_point',PointStamped,click_callback)
#goal_pub = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
#action_goal_pub = rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=1)
#cancel_goal_pub = rospy.Publisher('move_base/cancel',GoalID,queue_size=1)
goal_status_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,status_callback)
#goal_status_pub = rospy.Publisher('/move_base/result',MoveBaseActionResult,queue_size=1)
file_name="multi_goals";
json_file.json_clear(file_name)
rospy.spin()
