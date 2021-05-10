import time
import rospy
import os
import subprocess as sp

from multiprocessing.connection import Client, Listener
from std_msgs.msg import String

# Flask server process.
server_script_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "flask_server.py")
server_process = None

# Implement a full duplex connection with the flask server.
sender_address = ('localhost', 7000)
receiver_address = ('localhost', 7001)

server_sender = None
server_receiver_listener = None
server_receiver = None


def initialize_server_process():
    server_process = sp.Popen(["/usr/bin/python3", server_script_path])


def initialize_server_connection():
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


def cleanup():
    server_sender.send("Close")
    # Give the server time to close itself.
    time.sleep(1)
    server_sender.close()
    server_process.kill()

    
def main():
    rospy.init_node('hri', anonymous=True)

    initialize_server_process()
    initialize_server_connection()
    
    rospy.spin()

    cleanup()
    

if __name__ == '__main__':
    main()
