import base64
import cv2
import time
import rospy
import os
import threading
import subprocess as sp

from cv_bridge import CvBridge, CvBridgeError
from multiprocessing.connection import Client, Listener
from sensor_msgs.msg import Image
from std_msgs.msg import String

from hri.msg import RobotStatus

channels = [
    "ActionQueue",
    "SystemHealth",
    "RobotStatus",
    "ActiveModules",
    "RobotMessage",
    "RobotFace"
]

# Flask server process.
server_script_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "flask_server.py")
server_process = None

# Implement a full duplex connection with the flask server.
sender_address = ('localhost', 7000)
receiver_address = ('localhost', 7001)

server_sender = None
server_receiver_listener = None
server_receiver = None

# Message receiving from flask is performed in a different thread.
flask_messaging_thread = None

# OpenCV bridge.
bridge = CvBridge()

def initialize_server_process():
    global server_process
    server_process = sp.Popen(["/usr/bin/python3", server_script_path])


def initialize_server_connection():
    global server_sender, server_receiver
    # Give the subprocess some time to initialize its connection listener.
    time.sleep(1)
    # Create sender.
    server_sender = Client(sender_address)

    # Create a listener.
    server_receiver_listener = Listener(receiver_address)

    # Tell the server process that it can now start a sender client.
    # NOTE: the server should wait a little until the next line is
    # executed, otherwise the operation will fail.
    server_sender.send("CreateSender")

    server_receiver = server_receiver_listener.accept()


def flask_receive_handler():
    while True:
        if server_receiver is None:
            # Don't block execution.
            time.sleep(0.01)
            continue
        if server_receiver.poll():
            try:
                message = server_receiver.recv()
                if message == "shutdown":
                    rospy.loginfo("Stopping system")
                rospy.loginfo(message)
            except:
                break
        # Don't block execution.
        time.sleep(.01)

def robot_info_receive_handler(robot_status):
    if server_sender is not None:
        for channel in channels:
            try:
                server_sender.send({
                    "channel": channel,
                    "value": getattr(robot_status, channel)
                })
            except:
                print("Error fetching: ", channel)

def robot_video_feed_receive_handler(image_msg):
    if server_sender is not None:
        img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        _, img_buffer = cv2.imencode('.jpg', img)
        img_text = base64.b64encode(img_buffer)
        server_sender.send({
            "channel": "CameraFeed",
            "value": img_text.decode("ascii")
        })
        
    
def cleanup():
    print("Cleaning up")
    server_sender.send("Close")
    # Give the server time to close itself.
    time.sleep(1)
    server_sender.close()
    server_process.kill()

    
def main():
    rospy.init_node('hri', anonymous=True)

    initialize_server_process()
    initialize_server_connection()

    # Create thread that handles messages from flask.
    # The thread is daemonic so there's no need to clean
    # it up at the end of execution.
    flask_messaging_thread = threading.Thread(target=flask_receive_handler, daemon=True)
    flask_messaging_thread.start()

    # Start receiving status messages.
    rospy.Subscriber("/robot_info", RobotStatus, robot_info_receive_handler)
    # Start receiving video feed.
    rospy.Subscriber("/robot_video_feed", Image, robot_video_feed_receive_handler)
    
    rospy.spin()

    cleanup()
    

if __name__ == '__main__':
    main()
