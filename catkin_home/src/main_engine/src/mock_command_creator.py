#!/usr/bin/env python
# license removed for brevity

# DESCRIPTION
"""
    This script simulates a cmd id with a random number, 
    then sleeps a random time of seconds as well.
"""

# TOPICS PUBLISHED
"""
    engine_commands -> Byte
"""

# TOPICS READ
"""
    NA
"""


import rospy
from std_msgs.msg import Byte
import random

# Get a random comand id and random sleep seconds
SLEEP_MAX = 7
SLEEP_MIN = 1
CMD_MAX = 5
CMD_MIN = 1
def randomize_commands():
    sleep_seconds = random.randint(SLEEP_MIN,SLEEP_MAX) # Hz
    command_id = random.randint(CMD_MIN, CMD_MAX)

    return sleep_seconds, command_id

def talker():
    pub = rospy.Publisher('engine_commands', Byte, queue_size=10)
    rospy.init_node('Commander', anonymous=False)

    while not rospy.is_shutdown():
        sleep_seconds, command_id = randomize_commands()
        id_msg_str = "Published to engine with cmd id: %d" % command_id
        sleep_msg_str = "Sleeping %d seconds" % sleep_seconds
        # publish
        pub.publish(command_id)

        # log
        rospy.loginfo(id_msg_str)
        rospy.loginfo(sleep_msg_str)
        rospy.sleep(sleep_seconds) 

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass