#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
from intercom.msg import action_selector_cmd, bring_something_cmd
from object_detector.msg import DetectObjects3DAction, DetectObjects3DGoal  
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

START_STATE = "start"
SPEECH_STATE = "speech"
NAVIGATION1_STATE = "navigation1"
OBJECTDETECTION_STATE = "object_detection"
NAVIGATION2_STATE = "navigation2"
END_STATE = "done"

ARRIVE_NAVIGATION = "arrive"
RETURN_NAVIGATION = "return"
DONE_NAVIGATION = "done"

class MoveGoals(Enum):
    KITCHEN = 1
    COUCH = 2
    BATHROOM = 3
    CLOSET = 4

class Tmr2022Main(object):
    currentState =  START_STATE
    targetPlace = None
    targetObject = None

    def __init__(self):
        # Conversation/Speech
        self.speech_enable = rospy.Publisher("inputAudioActive", Bool, queue_size=10)
        self.parser_listener = rospy.Subscriber('action/bring_something', bring_something_cmd, self.listen_parser)
        
        # Navigation
        self.initial_pose = None
        rospy.loginfo("Waiting for MoveBase AS...")
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("MoveBase AS Loaded ...")
        self.initial_pose_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initial_pose_cb)
        self.nav_nav_client = actionlib.SimpleActionClient('navServer', navServAction)
        self.nav_client.wait_for_server()

        # Vision
        self.vision2D_enable = rospy.Publisher("detectionsActive", Bool, queue_size=10)

        self.run()

    def initial_pose_cb(self, msg):
        self.initial_pose = PoseStamped(header = msg.header, pose = msg.pose.pose)

    def run(self):
        self.currentState = SPEECH_STATE
        while (self.currentState == START_STATE) and not rospy.is_shutdown():
            pass
        self.speech_enable.publish(Bool(False))

    def nav_goal(self, target = MoveGoals.KITCHEN):
        class NavGoalScope:
            target_location = target.name
            result = False
            pose = PoseStamped()
            
            result_received = False
        
        def nav_goal_feedback(feedback_msg):
            NavGoalScope.pose = feedback_msg.pose
        
        def get_result_callback(state, result):
            NavGoalScope.result = result.result

            NavGoalScope.result_received = True
            rospy.loginfo("Nav Goal Finished")

        rospy.loginfo("Sending Nav Goal")
        self.nav_client.send_goal(
                    navServGoal(target_location = NavGoalScope.target_location),
                    feedback_cb=nav_goal_feedback,
                    done_cb=get_result_callback)
        
        while not NavGoalScope.result_received:
            pass
        
        return NavGoalScope.result

    def back_to_origin(self):
        goal = MoveBaseGoal()
        self.move_client.send_goal(self.initial_pose)
        self.move_client.wait_for_result()

    def listen_parser(self):
        ## Received Cmd
        self.targetPlace = bring_something_cmd.place
        self.targetObject = bring_something_cmd.object
        rospy.loginfo("Parser Received " + bring_something_cmd.place + "-" + bring_something_cmd.object)

        # GOTO-NAV
        ## TODO: PASAR DE STRING A ID
        self.nav_goal(MoveGoals(1))
        self.currentState = NAVIGATION1_STATE

        # 15 Seconds Vision Enable
        start = time.time()
        self.vision2D_enable.publish(Bool(True))
        while time.time() - start < 15:
            pass
        self.vision2D_enable.publish(Bool(False))

        ## GO BACK TO ORIGIN
        self.back_to_origin()


def main():
    rospy.init_node('Tmr2022Main', anonymous=True)
    rospy.loginfo("Tmr2022Main initialized.")
    Tmr2022Main()
    rospy.spin()

if __name__ == '__main__':
    main()