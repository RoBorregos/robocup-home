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

# MSG USED
"""
    action_selector_msg
        Header header
        uint8 cmd_id
        uint8 cmd_priority
        uint8 critic_shutdown
"""

import rospy
from intercom.msg import action_selector_cmd
import random

# Get a random comand id and random sleep seconds
SLEEP_MAX = 7
SLEEP_MIN = 1
CMD_MAX = 5
CMD_MIN = 1

def randomize_command():
    msg = action_selector_cmd()

    msg.cmd_id = random.randint(CMD_MIN, CMD_MAX)
    msg.cmd_priority = 0

    # Add 1/15 posibility of critic shutdown
    isCritic = (0,1)[random.randint(1, 10) == 2] # picking a random number match 2
    if (isCritic == 1):
        rospy.logfatal("Critic shutdown signal from Action Selector sent")
    msg.critic_shutdown = isCritic
    
    sleep_seconds = random.randint(SLEEP_MIN,SLEEP_MAX) # Hz

    return sleep_seconds, msg

def talker():
    pub = rospy.Publisher('engine_commands', action_selector_cmd, queue_size=10)
    rospy.init_node('Commander', anonymous=False)

    while not rospy.is_shutdown():
        # Generate a random msg
        sleep_seconds, msg = randomize_command()
        id_msg_str = "Published to engine with cmd id: %d" % msg.cmd_id
        sleep_msg_str = "Sleeping %d seconds" % sleep_seconds
        # publish
        pub.publish(msg)

        # log
        rospy.loginfo(id_msg_str)
        rospy.loginfo(sleep_msg_str)
        rospy.sleep(sleep_seconds) 

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass