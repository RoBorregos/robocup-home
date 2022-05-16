#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from intercom.msg import action_selector_cmd, bring_something_cmd
from object_detector.msg import DetectObjects3DAction, DetectObjects3DGoal  

class Tmr2022Main(object):
    targetPlace = None
    targetObject = None

    def __init__(self):
        # Conversation/Speech
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.parser_listener = rospy.Subscriber('action/bring_something', bring_something_cmd, self.listen_parser)
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)
        
        # Vision
        self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)
        self.vision3D_as = actionlib.SimpleActionClient("Detect3D", DetectObjects3DAction)
        rospy.loginfo("Waiting for ComputerVision 3D AS...")
        self.vision3D_as.wait_for_server()

        self.get_objects()

    def run(self):
        pass

    def listen_parser(self):
        self.targetPlace = bring_something_cmd.place
        self.targetObject = bring_something_cmd.object
        rospy.loginfo("Parser Received " + bring_something_cmd.place + "-" + bring_something_cmd.object)

    def get_objects(self):
        class GetObjectsScope:
            objects_found_so_far = 0
            objects_found = 0
            objects_poses = []
            objects_names = []
            objects_ids = []

            x_plane = 0.0
            y_plane = 0.0
            z_plane = 0.0
            width_plane = 0.0
            height_plane = 0.0
            result_received = False
        
        def get_objects_feedback(feedback_msg):
            GetObjectsScope.objects_found_so_far = feedback_msg.status
        
        def get_result_callback(state, result):
            GetObjectsScope.objects_poses = result.objects_poses
            GetObjectsScope.objects_names = result.objects_names
            GetObjectsScope.objects_found = result.objects_found
            GetObjectsScope.objects_ids = result.objects_ids
            GetObjectsScope.x_plane = result.x_plane
            GetObjectsScope.y_plane = result.y_plane
            GetObjectsScope.z_plane = result.z_plane
            GetObjectsScope.width_plane = result.width_plane
            GetObjectsScope.height_plane = result.height_plane
            GetObjectsScope.result_received = True
            rospy.loginfo("Objects Received: " + str(GetObjectsScope.objects_found))

        rospy.loginfo("Getting objects")
        self.vision3D_as.send_goal(
                    DetectObjects3DGoal(),
                    feedback_cb=get_objects_feedback,
                    done_cb=get_result_callback)
        
        while not GetObjectsScope.result_received:
            pass
        
        object_poses = GetObjectsScope.objects_poses
        object_names = GetObjectsScope.objects_names
        object_ids = GetObjectsScope.objects_ids
        self.objects = zip(object_ids, object_names, object_poses)

def main():
    rospy.init_node('Tmr2022Main', anonymous=True)
    rospy.loginfo("Tmr2022Main initialized.")
    Tmr2022Main()
    rospy.spin()

if __name__ == '__main__':
    main()