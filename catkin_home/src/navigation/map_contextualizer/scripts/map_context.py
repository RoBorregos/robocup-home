#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

from __future__ import print_function

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

server = None
marker_pos = 0

menu_handler = MenuHandler()

h_first_entry = 0
roi_dict = {}

def makeBox( msg ):

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( int_marker )
    int_marker.controls.append(control)

    server.insert( int_marker )

def savePose(id, x, y, z, w):
    return

def deepCb( feedback ):
    EntryContext context = menu_handler.entry_contexts_[feedback.menu_entry_id]
    rospy.loginfo ("Insert ROI => ROI Name: " + context.title + "   ROI ID: " + roi_dict[context.title] + "    POSE: x = {} y = {} z = {} w = {}" feedback.pose.x, feedback.pose.y, feedback.pose.z, feedback.pose.w)
    savePose(roi_dict[context.title], feedback.pose.x, feedback.pose.y, feedback.pose.z, feedback.pose.w )


def initMenu():
    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert( "Insert ROI Entry" )
    for roi_name, roi_id in roi_dict:
        entry = menu_handler.insert( str(roi_name), parent=h_first_entry, callback=deepCb);

if __name__=="__main__":
    rospy.init_node("map_roi")
    
    server = InteractiveMarkerServer("map_roi")

    initMenu()
    
    makeMenuMarker( "robot" )

    menu_handler.apply( server, "robot" )
    server.applyChanges()

    rospy.spin()

    # Existen dos tipos de implementaicon, la priemra es utilizar el menu_handler con el parametro feedback y guardar las poses en un csv, 
    # La segunda es hacer un menu handler que al momento de hacer un callback, se saca el ultimo nav goal de un queue y se guarda esa pose en un csv
