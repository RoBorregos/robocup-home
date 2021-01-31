#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import tf
from std_msgs.msg import Int16
from map_visualizer.msg import MapDisplayState
from map_visualizer.msg import MapContext

markerArray = MarkerArray()
marker_state = MapDisplayState()

map_colors = ((1.0, 0.0, 1.0), (0.0, 1.0, 0.0))

marker_map = MapContext()

##################################################
# State 0 = wating/default
# State 1 = room
# State 2 = obj of interest
##################################################

def getMapContext(data):
    global marker_state
    global marker_map
    if(marker_state.state not in [-1]):
        marker_map = data
        rospy.loginfo("Map obtained and updated")
    else:
        marker_map = None

def changeState(data):
    global marker_state
    global marker_map
    marker_state = data
    if marker_state.state == 0:
        #display all the map
        rospy.loginfo("Displaying the whole map by default")
        marker_state.room = ""
    elif marker_state.state == 1:
        #display specified room
        rospy.loginfo("Displaying specific room emphasized")
    elif marker_state.state == 2:
        rospy.loginfo("Displaying specific room with objects")
    rospy.loginfo("State changed to " + str(marker_state.state))

def updateMarkers():
    global markerArray
    global marker_map
    global marker_state
    markerArray = MarkerArray()
    counter=0
    if(marker_map != None):
        for room in marker_map.rooms:
            area_counter = 0
            for area_point in room.area:
                br.sendTransform((area_point.x, area_point.y, area_point.z),
                                (0.0, 0.0, 0.0, 1.0),
                                rospy.Time.now(),
                                room.name + "-" + str(area_counter),
                                "/map")
                marker = Marker()
                marker.header.frame_id = room.name + "-" + str(area_counter)
                marker.header.stamp = rospy.Time.now()
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.ns = "map_goal_point"
                marker.id = counter
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                map_color = 0
                if(room.name == marker_state.room):
                    map_color = 1
                marker.color.a = 1.0
                marker.color.r = map_colors[map_color][0]
                marker.color.g = map_colors[map_color][1]
                marker.color.b = map_colors[map_color][2]
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.lifetime = rospy.Duration.from_sec(0.1)
                markerArray.markers.append(marker)
                area_counter+=1
                counter+=1
            area_counter = 0
            for obj in room.obj_int:
                br.sendTransform((obj.obj_area[0].x, obj.obj_area[0].y, obj.obj_area[0].z),
                                (0.0, 0.0, 0.0, 1.0),
                                rospy.Time.now(),
                                room.name + "-" + obj.name + "-" + str(area_counter),
                                "/map")
                marker = Marker()
                marker.header.frame_id = room.name + "-" + obj.name + "-" + str(area_counter)
                marker.header.stamp = rospy.Time.now()
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.ns = "map_goal_point"
                marker.id = counter
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                map_color = 0
                if(room.name == marker_state.room):
                    map_color = 1
                marker.color.a = 1.0
                marker.color.r = map_colors[map_color][0]
                marker.color.g = map_colors[map_color][1]
                marker.color.b = map_colors[map_color][2]
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                marker.lifetime = rospy.Duration.from_sec(0.1)
                markerArray.markers.append(marker)
                area_counter+=1
                counter+=1
    # rospy.loginfo("Map updated")

if __name__ == '__main__':
    rospy.Subscriber("/tagged_map", MapContext, getMapContext)
    rospy.Subscriber("/map_tagger_state", MapDisplayState, changeState)
    topic = 'goal_visualization_array'
    publisher = rospy.Publisher(topic, MarkerArray)
    br = tf.TransformBroadcaster()
    rospy.init_node('map_displayer')

    # global path
    while not rospy.is_shutdown():
        counter=0
        updateMarkers()
        global markerArray
        publisher.publish(markerArray)
        rospy.sleep(0.01)