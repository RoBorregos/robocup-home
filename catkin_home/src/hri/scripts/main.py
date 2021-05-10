import time
import rospy
import os
import subprocess as sp

from multiprocessing.connection import Client
from std_msgs.msg import String

# Connection to the flask server.server_connection = None

def main():
    rospy.init_node('hri', anonymous=True)

    # Create a new process to execute the flask server.
    server_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "flask_server.py")
    p = sp.Popen(["/usr/bin/python3", server_path])

    address = ('localhost', 7000)
    # Give the subprocess some time to initialize its connection listener.
    time.sleep(1)
    server_connection = Client(address)

    rospy.spin()

    # Cleanup the flask server process.
    server_connection.close()
    p.kill()
    rospy.loginfo("Killed web server")
    

if __name__ == '__main__':
    main()
