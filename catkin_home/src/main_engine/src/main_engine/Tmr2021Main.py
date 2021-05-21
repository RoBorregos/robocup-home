#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from intercom.msg import action_selector_cmd, bring_something_cmd
from object_detector.msg import ObjectArray, ObjectDetected

START_STATE = "start"
SPEECH_STATE = "speech"
NAVIGATION1_STATE = "navigation1"
OBJECTDETECTION_STATE = "object_detection"
NAVIGATION2_STATE = "navigation2"
END_STATE = "done"

ARRIVE_NAVIGATION = "arrive"
RETURN_NAVIGATION = "return"
DONE_NAVIGATION = "done"

class Tmr2021Main(object):
    currentState =  START_STATE
    targetPlace = None
    targetObject = None

    def __init__(self):
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.parser_listener = rospy.Subscriber('action/bring_something', bring_something_cmd, self.listen_parser)
        
        self.talkerDashgo = rospy.Publisher("navBridgeServer/listener", String, queue_size=20)
        self.listenerDashgo = rospy.Subscriber("navBridgeServer/talker", String, self.listen_dashgo)
        
        self.status_objects = rospy.Subscriber("objects_detected", ObjectArray, self.listen_object_detection)
        
        self.say_publisher = rospy.Publisher('robot_text', String, queue_size=10)

        self.run()

    def listen_dashgo(self, msg):
        rospy.loginfo("Message Dashgo Received")
        if msg.data == ARRIVE_NAVIGATION:
            rospy.loginfo("CHANGE STATE: " + OBJECTDETECTION_STATE)
            self.currentState = OBJECTDETECTION_STATE
        if msg.data == RETURN_NAVIGATION:
            self.currentState = NAVIGATION2_STATE
            rospy.loginfo("CHANGE STATE: " + NAVIGATION2_STATE)
        if msg.data == DONE_NAVIGATION:
            self.currentState = END_STATE
            rospy.loginfo("CHANGE STATE: " + END_STATE)
            self.say_publisher.publish("Here is the " + self.targetObject + " take it.")

    def listen_object_detection(self, objectArray):
        if self.currentState == OBJECTDETECTION_STATE:
            for objectDetected in objectArray:
                if objectDetected.id.lower() == self.targetObject:
                    rospy.loginfo("Object found")
                    self.talkerDashgo.publish(DONE_NAVIGATION)
                    self.currentState = NAVIGATION2_STATE
                    rospy.loginfo("CHANGE STATE: " + NAVIGATION2_STATE)

    def listen_parser(self, bring_something_cmd):
        if self.currentState != SPEECH_STATE and self.currentState != START_STATE:
            return
        
        self.targetPlace = bring_something_cmd.place
        self.targetObject = bring_something_cmd.object
        rospy.loginfo("Parser Received " + bring_something_cmd.place + "-" + bring_something_cmd.object )
        self.talkerDashgo.publish(self.targetPlace)
        self.currentState = NAVIGATION1_STATE
        rospy.loginfo("CHANGE STATE: " + NAVIGATION1_STATE)

    def run(self):
        self.currentState == SPEECH_STATE
        while (self.currentState == START_STATE or self.currentState == SPEECH_STATE) and not rospy.is_shutdown():
            pass
        self.speech_enable.publish(Bool(False))

def main():
    rospy.init_node('Tmr2021Main', anonymous=True)
    rospy.loginfo("Tmr2021Main initialized.")
    Tmr2021Main()
    rospy.spin()

if __name__ == '__main__':
    main()