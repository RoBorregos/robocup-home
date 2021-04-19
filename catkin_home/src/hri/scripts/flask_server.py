import flask
import eventlet
import cv2
from multiprocessing.connection import Listener

from flask import request, jsonify
from flask_socketio import SocketIO, send
from flask_cors import CORS

app = flask.Flask(__name__)
app.config["SCHEDULER_API_ENABLED"] = True
CORS(app)

socketio = SocketIO(app, cors_allowed_origins="*")

# Establish connection with the HRI ROS node process.
address = ('localhost', 7000)
ros_listener = Listener(address)
ros_connection = ros_listener.accept()

# Forward all messages to "Comm" websocket endpoint.
def message_forwarder():
    while True:
        # Only call `recv` when we're sure there's a new message since
        # it is a blocking call.
        if(ros_connection.poll()):
            socketio.emit("Comm", ros_connection.recv())
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

    socketio.start_background_task(message_forwarder)
    
    # Do not use default port 5000, it might be used by ROS or other os service.
    socketio.run(app, use_reloader=False, port=5050)