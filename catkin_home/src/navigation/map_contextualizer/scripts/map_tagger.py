#!/usr/bin/env python3
""" Map tagger application script.

Application for manual map contextualization done by tagging the map,
with rooms and objects of interest.

The user will use rviz and the terminal to define the area of rooms 
and objects of interest in the map. For this, please run the map_tagger
launch file and follow the app menu.

Global app states:
-1: Tagging finished
 0: Waiting state
 1: Add room area
 2: Add object area

In the end, it generates a .json file with the map description.
"""
import pathlib
import rospy
from geometry_msgs.msg import PoseStamped
import json
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from map_contextualizer.msg import MapDisplayState
from map_contextualizer.msg import MapContext
from map_contextualizer.msg import Room
from map_contextualizer.msg import ObjInt
#from map_contextualizer.scripts import map
import sys 
sys.path.append(str(pathlib.Path(__file__).parent) + 
'/home/bryan/robocup-home/catkin_home/src/navigation/map_contextualizer/scripts')
from map import *
import os
from os import listdir
from os.path import isfile, join
global map_publisher
global state_publisher
global state
global obj_name
#global map

def callback(data):
    """Gets point location placed on the map in rviz and
    adds it to the map msg according to its state

    Parameters
    ----------
    data : PoseStamped
        /move_base_simple/goal published by 2D Nav Goal.
    """

    # rospy.loginfo(rospy.get_caller_id() + "Pose given %s", data.pose.position)
    global map
    global map_publisher
    global obj_name
    if(state.state not in [0, -1]):
        room_index = -1
        for index in range(len(map.rooms)):
                if map.rooms[index].name == state.room:
                   room_index = index
        if state.state == 1:
            map.rooms[room_index].entrances.append(data.pose.position)
        elif state.state == 2:
            map.rooms[room_index].area.append(data.pose.position)
        elif state.state == 3:
            map.rooms[-1].path.append(data.pose.position)
        elif state.state == 4:
            map.rooms[-1].obj_int[-1].obj_area.append(data.pose.position)
        map_publisher.publish(map)
    
state = MapDisplayState()
map = MapContext()
map_publisher = None
state_publisher = None
obj_name = None

if __name__ == '__main__':
    """Main application workflow.
    Follows steps to save map data, publishes /tagged_map and
    /map_tagger_state topics each time the map is tagged.
    When finished, generates a json with the information.

    User app states:
    0: Finished application
    1: Add room (area, obj of interest)
    2: Delete room
    """
    
    #global map_publisher
    #global state_publisher
    rospy.init_node('map_tagger')
    topic = 'tagged_map'
    map_publisher = rospy.Publisher("tagged_map", MapContext)
    state_publisher = rospy.Publisher("map_tagger_state", MapDisplayState)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)

    rospy.loginfo("Map tagger v1")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~")
    #global map
    #global state
    print("What do you want to do?")
    print("1. Open existing map")
    print("2. Create new map")
    print("0. Exit")
    x = int(input())
    print("")

    if(x not in [1, 2]):
    #if(x == 0):
        exit(0)

    if(x==1):
        print("Choose a map:")
        maps = [f for f in listdir("/home/bryan/robocup-home/catkin_home/src/navigation/map_contextualizer/contextmaps") ]
        x=-1
        for index, context_map in enumerate(maps):
            print((str(index+1) + ". " + context_map[:-5]))
        print("0. Exit")
        x = int(input())
        if x==0 or x>len(maps): exit(0)

        context_map = None
        with open("/home/bryan/robocup-home/catkin_home/src/navigation/map_contextualizer/contextmaps" + maps[x-1], "r") as read_file:
            context_map = json.load(read_file)

        map.name = context_map["name"]
        for room in context_map["rooms"]:
            new_room = Room()
            new_room.name = context_map["rooms"][room]["name"]
            map.rooms.append(new_room)
            for area_point in context_map["rooms"][room]["area"]:
                point = Point()
                point.x = area_point[0]
                point.y = area_point[1]
                point.z = 0.0
                map.rooms[-1].area.append(point)
            for path_point in context_map["rooms"][room]["path"]:
                point = Point()
                point.x = path_point[0]
                point.y = path_point[1]
                point.z = 0.0
                map.rooms[-1].path.append(point)
            for entrance in context_map["rooms"][room]["entrance"]:
                point = Point()
                point.x = entrance[0]
                point.y = entrance[1]
                point.z = 0.0
                map.rooms[-1].entrances.append(point)
            for obj in context_map["rooms"][room]["obj_int"]:
                obj_int = ObjInt()
                obj_int.name = obj
                map.rooms[-1].obj_int.append(obj_int)
                for obj_point in context_map["rooms"][room]["obj_int"][obj]:
                    point = Point()
                    point.x = obj_point[0]
                    point.y = obj_point[1]
                    point.z = 0.0
                    map.rooms[-1].obj_int[-1].obj_area.append(point)
        map_publisher.publish(map)
        print(map)
    else:
        x = input("Give me the name of the map: ")
        map.name = x

    while not rospy.is_shutdown():
        print("What do you want to do?")
        print("1. Add a room")
        print("2. Delete a room")
        print("0. Exit")
        x = int(input())
        if x == 0:
            break
        if x == 1:
            # TODO: Change all inputs to input for python3
            room_name = str(input('Give me the name of the room: '))
            state.state = 1
            state.room = room_name
            state_publisher.publish(state)
            new_room = Room()
            new_room.name = room_name
            map.rooms.append(new_room)
            print("Please choose the room's entrance(s)")
            print("Insert any key to stop")
            x = input()
            state.state = 2
            print("Please choose the enclosing perimeter area of the room with the goal publisher")
            print("Insert any key to stop")
            x = input()
            print("Area saved.")
            state.state = 3
            print("Please make a path around the room with the goal publisher")
            print("Insert any key to stop")
            x = input()
            print("Path saved.")
            state.state = 0
            x = int(input("Save object of interest? (0 = no, 1 = Yes): "))
            while(x):
                #global obj_name
                obj_name = str(input('Give me the name of the object: '))
                state.state = 4
                state_publisher.publish(state)
                obj = ObjInt()
                obj.name = obj_name
                map.rooms[-1].obj_int.append(obj)
                print("Please choose the enclosing perimeter area of the object")
                print("Insert any key to stop")
                x = input()
                x = int(input("Save another object of interest? (0 = no, 1 = Yes): "))
            state.state = 0
            state_publisher.publish(state)
        elif x == 2:
            state.state = 0
            state_publisher.publish(state)
            print("Which room do you want to delete?")
            for index in range(len(map.rooms)):
                print((str(index+1) + ". " + map.rooms[index].name))
            print("0. Exit")
            x = int(input())
            while x <= len(map.rooms) and x > 0:
                room_deleted = map.rooms.pop(x-1)
                map_publisher.publish(map)
                print(("Room " + room_deleted.name + " deleted."))
                print("\nWhich room do you want to delete?")
                for index in range(len(map.rooms)):
                    print((str(index+1) + ". " + map.rooms[index].name))
                print("0. Exit")
                x = int(input())
    state.state = -1
    state_publisher.publish(state)
    map_obj = Map(map=map)
    print((map_obj.toJSON()))
    cur_path = os.path.dirname(__file__)
    with open(cur_path + "/../contextmaps/" + str(map.name) + '.json', "w") as f:
        f.write(map_obj.toJSON())
