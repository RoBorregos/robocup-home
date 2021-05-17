#!/usr/bin/env python3

import roslaunch
import os

def main():
    # For this challenge just launch speech to text and say.py
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print(os.getcwd())
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["./src/action_selectors/launch/speech_to_text.launch"])
    launch.start()

if __name__=="__main__":
    main()