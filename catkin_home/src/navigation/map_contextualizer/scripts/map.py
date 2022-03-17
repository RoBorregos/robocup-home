#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import json

""" Map tagger classes, where the class Map contains an array of rooms.

Inside rooms, objects of interest, paths, entrances and its overal area
can be defined. 
"""

class Room:
    """Room class to store room characteristics and contain objects positions."""
    def __init__(self, name):
        self.area = []
        self.obj_int = {}
        self.path = []
        self.name = name
        self.entrance = []

    def __init__(self, name, area, path, obj_int, entrances):
        self.name = name
        self.area = []
        self.obj_int = {}
        self.path = []
        self.entrance = []
        self.set_entrance(entrances)
        self.set_area(area)
        self.set_path(path)
        for index in range(len(obj_int)):
            self.set_obj_int(obj_int[index])

    def set_area(self, points):
        for point in points:
            self.area.append([point.x, point.y, point.z])

    def set_path(self, points):
        for point in points:
            self.path.append([point.x, point.y, point.z])

    def set_obj_int(self, obj):
        self.obj_int[obj.name] = []
        for point in obj.obj_area:
            self.obj_int[obj.name].append([point.x, point.y, point.z])

    def set_entrance(self, points):
        for point in points:
            self.entrance.append([point.x, point.y, point.z])

    def reset_room(self):
        self.area = []
        self.obj_int = []
        self.path = []
        self.name = name
        self.entrance = None

class Map:
    """Map class that can contain rooms."""
    def __init__(self, name):
        self.rooms = {}
        self.name = name

    def __init__(self, map):
        self.rooms = {}
        self.name = map.name
        self.set_rooms(map.rooms)

    def set_rooms(self, room_list):
        for room in room_list:
            self.rooms[room.name]=Room(room.name, room.area, room.path, room.obj_int, room.entrances)

    def toJSON(self):
        return json.dumps(self, default=lambda o: o.__dict__, 
            sort_keys=True, indent=4)
        