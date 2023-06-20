#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoorDetector:
    def __init__(self):
        self.last_distance = None
        self.door_opened = False

        self.firstIteration = True
        # Subscribe to the LiDAR data topic
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        # Create a publisher for the movement command topic
        self.cmd_pub = rospy.Publisher('/move_command', String, queue_size=10)

    def lidar_callback(self, msg):
        # Filter the LiDAR data to only include points near the door
        door_angle = 1.6  # Replace with the angle of the door in the LiDAR data (rad) 1.6 is straight of our robot
        #door_distance = 0.3  # Replace with the distance of the door in the LiDAR data (m)
        door_idx = int(door_angle / msg.angle_increment)
        min_idx = max(0, door_idx - 10)
        max_idx = min(len(msg.ranges), door_idx + 10)
        door_ranges = msg.ranges[min_idx:max_idx]

        # Calculate the median distance to the door
        door_distance = sorted(door_ranges)[len(door_ranges)//2]
        #rospy.loginfo("Door: %f", door_distance)
        # Check if the distance has changed significantly since the last scan
        if self.last_distance is not None and abs(door_distance - self.last_distance) > 0.30:   #> #.## (m) how much difference from door_distance
            self.door_opened = True
        self.last_distance = door_distance

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.door_opened:
                # Send a message to the robot's control node to start moving
                # cmd_msg = String()
                # cmd_msg.data = "start_moving"
                # self.cmd_pub.publish(cmd_msg)
                return True
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('door_detector')
    detector = DoorDetector()
    detector.run()