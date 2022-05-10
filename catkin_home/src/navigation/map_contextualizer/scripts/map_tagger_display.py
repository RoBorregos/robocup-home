#!/usr/bin/env python3
""" Map tagger display script.

Displays the map information given by the /tagged_map topic by
converting it into transforms and placing markers in each one.

Global app states:
-1: Tagging finished
0:  Waiting state
1:  Add room area
2:  Add object area
"""

import tf2_ros
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
from std_msgs.msg import Int16
from map_contextualizer.msg import MapDisplayState
from map_contextualizer.msg import MapContext

markerArray = MarkerArray()
marker_state = MapDisplayState()
marker = Marker()

# Green for active, purple for inactive
map_colors = ((1.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1, 0.2, 0.46))

marker_map = MapContext()

counter = 0
area_counter = 0

##################################################
# State 0 = wating/default
# State 1 = room
# State 2 = obj of interest
##################################################

def getMapContext(data):
    """Updates map context from /tagged_map topic.

    Parameters
    ----------
    data : MapContext
        /tagged_map published by user app.
    """

    global marker_state
    global marker_map
    if(marker_state.state not in [-1]):
        marker_map = data
        rospy.loginfo("Map obtained and updated")
    else:
        marker_map = None

def changeState(data):
    """Updates app state based on /map_tagger_state topic.

    Parameters
    ----------
    data : MapDisplayState
        Global app state published by user app.
    """

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

def defineMarker(current_room, frame_id, marker_type):
    """Defines marker creating a transform and attaching it to its frame.

    Parameters
    ----------
    current_room : boolean
        Tells if room is the one being defined in the application.
    frame_id : string
        Unique frame id for marker and transform creation
    marker_type : int
        Variable that states the shape the object must have.
    """

    global marker
    global counter
    global area_counter
    
    marker = Marker()
    marker.header.frame_id = frame_id
    if marker_type == 0:
        marker.type = marker.CYLINDER
    elif marker_type == 1:
        marker.type = marker.SPHERE
    elif marker_type == 2:
        marker.type = marker.SPHERE
    else:
        marker.type = marker.CUBE
    marker.header.stamp = rospy.Time.now()
    marker.action = marker.ADD
    marker.ns = "map_goal_point"
    marker.id = counter
    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    map_color = 0
    if(marker_type == 2):
        map_color = 2
    elif(current_room):
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


def updateMarkers():
    """Updates map visualization in rviz.

    A transform base on the current Map context positions is 
    created and a marker is attached to it.

    - Cylinder = entrace
    - Sphere = room area
    - Cube = object of interest

    This must be updated frequently because markers have a
    lifetime of 0.1 seconds, so that they can also be eliminated
    if a room is eliminated.
    """

    global markerArray
    global marker_map
    global marker_state
    global marker
    global counter
    global area_counter
    markerArray = MarkerArray()
    counter=0
    if(marker_map != None):
        for room in marker_map.rooms:
            area_counter = 0
            for entrance in room.entrances:
                frame_id = room.name + "-entrance-" + str(area_counter)
                br.sendTransform((entrance.x, entrance.y, entrance.z),
                    (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), frame_id, "/map")
                defineMarker(room.name == marker_state.room, frame_id, 0)
            area_counter = 0
            for area_point in room.area:
                frame_id = room.name + "-" + str(area_counter)
                br.sendTransform((area_point.x, area_point.y, area_point.z),
                    (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), frame_id, "/map")
                defineMarker(room.name == marker_state.room, frame_id, 1)
            area_counter = 0
            for path_point in room.path:
                frame_id = room.name + "-path-" + str(area_counter)
                br.sendTransform((path_point.x, path_point.y, path_point.z),
                    (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), frame_id, "/map")
                defineMarker(room.name == marker_state.room, frame_id, 2)
            area_counter = 0
            for obj in room.obj_int:
                frame_id = room.name + "-" + obj.name + "-" + str(area_counter)
                br.sendTransform((obj.obj_area[0].x, obj.obj_area[0].y, obj.obj_area[0].z),
                    (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), frame_id, "/map")
                defineMarker(room.name == marker_state.room, frame_id, 3)
    # rospy.loginfo("Map updated")

if __name__ == '__main__':
    """ Main, updates transforms and markers. """

    rospy.Subscriber("/tagged_map", MapContext, getMapContext)
    rospy.Subscriber("/map_tagger_state", MapDisplayState, changeState)
    topic = 'goal_visualization_array'
    publisher = rospy.Publisher(topic, MarkerArray)
    br = tf2_ros.TransformBroadcaster()
    rospy.init_node('map_displayer')

    # global path
    while not rospy.is_shutdown():
        counter=0
        updateMarkers()
        #global markerArray
        publisher.publish(markerArray)
        rospy.sleep(0.01)