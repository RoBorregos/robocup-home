#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import json

class Room:
    def __init__(self, name):
        self.area = []
        self.obj_int = {}
        self.path = []
        self.name = name
        self.entrance = []

    def __init__(self, name, area, obj_int):
        self.name = name
        self.area = []
        self.obj_int = {}
        self.path = []
        self.entrance = []
        self.set_area(area)
        for index in range(len(obj_int)):
            self.set_obj_int(obj_int[index])

    def set_area(self, points):
        for point in points:
            self.area.append([point.x, point.y, point.z])

    def set_obj_int(self, obj):
        self.obj_int[obj.name] = []
        for point in obj.obj_area:
            self.obj_int[obj.name].append([point.x, point.y, point.z])

    def set_entrance(self, point):
        self.entrance = point

    def reset_room(self):
        self.area = []
        self.obj_int = []
        self.path = []
        self.name = name
        self.entrance = None

class Map:
    def __init__(self, name):
        self.rooms = {}
        self.name = name

    def __init__(self, map):
        self.rooms = {}
        self.name = map.name
        self.set_rooms(map.rooms)

    def set_rooms(self, room_list):
        for room in room_list:
            self.rooms[room.name]=Room(room.name, room.area, room.obj_int)

    def toJSON(self):
        return json.dumps(self, default=lambda o: o.__dict__, 
            sort_keys=True, indent=4)
        