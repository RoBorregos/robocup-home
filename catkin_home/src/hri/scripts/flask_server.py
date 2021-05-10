import flask
import eventlet
import cv2
from multiprocessing.connection import Listener, Client

from flask import request, jsonify
from flask_socketio import SocketIO, send
from flask_cors import CORS

app = flask.Flask(__name__)
app.config["SCHEDULER_API_ENABLED"] = True
CORS(app)

socketio = SocketIO(app, cors_allowed_origins="*")

# Full duplex connection with the HRI ros node.

receiver_address = ('localhost', 7000)
sender_address = ('localhost', 7001)

ros_receiver_listener = None
ros_receiver = None
ros_sender = None

def initialize_ros_sender():
    # Wait to give the ros node time to call its accept method.
    socketio.sleep(1)
    ros_sender = Client(sender_address)

def initialize_ros_receiver():
    ros_receiver_listener = Listener(receiver_address)
    ros_receiver = ros_receiver_listener.accept()

def cleanup():
    ros_sender.close()

    
# Forward all messages to "Comm" websocket endpoint.
def ros_receive_handler():
    while True:
        if ros_receiver is None:
            # To avoid blocking the process
            socketio.sleep(0.01)
            continue
        # Only call `recv` when we're sure there's a new message since
        # it is a blocking call.
        if(ros_receiver.poll()):
            message = ros_receiver.recv()
            if message == "CreateSender":
                initialize_ros_sender()
            elif message == "Close":
                cleanup()
            else:
                # TODO: this should be serialized/processed
                socketio.emit("Comm", message)
        socketio.sleep(0.1)

@socketio.on("connect")
def connect_handler():
    print('A user has connected to the server.')

@socketio.on("disconnect")
def disconnect_handler():
    print('A user has disconnected from the server.')

@socketio.on('message')
def handle_message(message):
    print(f"Received message {message}")
    send(message)

@app.errorhandler(404)
def not_found(_):
    return jsonify({"message": "The result was not found"}), 404

@app.route("/", methods=["GET"])
def root():
    socketio.emit("Comm", "Just some message")
    return jsonify({"message": "Welcome to the HOME API"})

if __name__ == "__main__":
    # Patch python to make use of an event loop and make background tasks work.
    eventlet.monkey_patch()

    initialize_ros_receiver()
    
    socketio.start_background_task(ros_receive_handler)
    
    # Do not use default port 5000, it might be used by ROS or other os service.
    socketio.run(app, use_reloader=False, port=5050)

